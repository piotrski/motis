#include "motis/rt/trip_progress_projection.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <list>
#include <map>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include "geo/polyline.h"

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/frun.h"
#include "nigiri/shapes_storage.h"
#include "nigiri/timetable.h"

#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {
namespace {

struct cached_run_shape {
  std::vector<geo::latlng> points_;
  std::vector<double> point_distances_;
  std::vector<std::size_t> stop_point_indices_;
  std::vector<double> stop_distances_;
  std::vector<unsigned> static_stop_sequences_;
};

struct projection_candidate {
  double lateral_error_;
  double distance_along_;
  std::size_t next_stop_idx_;
};

struct candidate_selection {
  trip_progress_projection_status status_;
  std::optional<projection_candidate> candidate_;
};

struct cache_key {
  n::trip_idx_t trip_;
  n::stop_idx_t from_;
  n::stop_idx_t to_;

  bool operator<(cache_key const& other) const {
    return std::tuple{cista::to_idx(trip_), cista::to_idx(from_),
                      cista::to_idx(to_)} <
           std::tuple{cista::to_idx(other.trip_), cista::to_idx(other.from_),
                      cista::to_idx(other.to_)};
  }
};

struct scheduled_trip_segment {
  n::trip_idx_t trip_;
  n::interval<n::stop_idx_t> range_;
};

std::optional<scheduled_trip_segment> get_trip_segment(n::rt::frun const& fr) {
  auto segment = std::optional<scheduled_trip_segment>{};
  auto ambiguous = false;
  fr.for_each_trip(
      [&](n::trip_idx_t const trip, n::interval<n::stop_idx_t> const range) {
        if (ambiguous || trip == n::trip_idx_t::invalid() ||
            fr.stop_range_.from_ < range.from_ ||
            fr.stop_range_.to_ > range.to_) {
          return;
        }
        if (segment.has_value() && segment->trip_ != trip) {
          ambiguous = true;
          return;
        }
        segment = scheduled_trip_segment{trip, range};
      });
  return ambiguous ? std::nullopt : segment;
}

std::optional<cached_run_shape> make_cached_shape(
    n::rt::frun const& fr,
    n::shapes_storage const& shapes,
    scheduled_trip_segment const& segment) {
  if (!fr.is_scheduled() || fr.size() < 2U) {
    return std::nullopt;
  }

  auto const trip = segment.trip_;
  auto const trip_range = segment.range_;

  if (trip >= shapes.trip_offset_indices_.size()) {
    return std::nullopt;
  }
  auto const [_shape_idx, offset_idx] = shapes.trip_offset_indices_[trip];
  if (offset_idx == n::shape_offset_idx_t::invalid() ||
      offset_idx >= shapes.offsets_.size()) {
    return std::nullopt;
  }

  auto const offsets = shapes.offsets_[offset_idx];
  auto const local_from =
      static_cast<n::stop_idx_t>(fr.stop_range_.from_ - trip_range.from_);
  auto const local_to =
      static_cast<n::stop_idx_t>(fr.stop_range_.to_ - trip_range.from_);
  if (local_to > offsets.size() || local_to <= local_from + 1U) {
    return std::nullopt;
  }

  auto const first_point = static_cast<unsigned>(offsets[local_from]);
  auto const last_point =
      static_cast<unsigned>(offsets[static_cast<n::stop_idx_t>(local_to - 1U)]);
  auto const full_shape = shapes.get_shape(trip);
  if (first_point >= full_shape.size() || last_point >= full_shape.size() ||
      first_point > last_point) {
    return std::nullopt;
  }

  auto const shape =
      shapes.get_shape(trip, n::interval<n::stop_idx_t>{local_from, local_to});
  if (shape.empty()) {
    return std::nullopt;
  }

  auto cached = cached_run_shape{};
  cached.points_.assign(begin(shape), end(shape));
  cached.point_distances_.reserve(cached.points_.size());
  cached.point_distances_.push_back(0.0);
  for (auto i = std::size_t{1U}; i != cached.points_.size(); ++i) {
    cached.point_distances_.push_back(
        cached.point_distances_.back() +
        geo::distance(cached.points_[i - 1U], cached.points_[i]));
  }

  auto previous_point = first_point;
  for (auto stop = local_from; stop != local_to; ++stop) {
    auto const absolute_point = static_cast<unsigned>(offsets[stop]);
    if (absolute_point < previous_point) {
      return std::nullopt;
    }
    previous_point = absolute_point;
    auto const point = absolute_point - first_point;
    if (point >= cached.point_distances_.size()) {
      return std::nullopt;
    }
    cached.stop_point_indices_.push_back(point);
    cached.stop_distances_.push_back(cached.point_distances_[point]);
  }

  auto const seq_range = n::loader::gtfs::stop_seq_number_range{
      {fr.tt_->trip_stop_seq_numbers_[trip]},
      static_cast<n::stop_idx_t>(trip_range.size())};
  auto all_sequences = std::vector<unsigned>{};
  for (auto const sequence : seq_range) {
    all_sequences.push_back(sequence);
  }
  if (local_to > all_sequences.size()) {
    return std::nullopt;
  }
  cached.static_stop_sequences_.assign(
      std::next(begin(all_sequences), local_from),
      std::next(begin(all_sequences), local_to));
  return cached;
}

std::optional<std::size_t> get_constrained_stop_idx(
    cached_run_shape const& shape,
    vehicle_position_progress_constraint const& constraint) {
  auto const seq = std::ranges::find(shape.static_stop_sequences_,
                                     constraint.current_static_stop_sequence_);
  if (seq == end(shape.static_stop_sequences_)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(
      std::distance(begin(shape.static_stop_sequences_), seq));
}

std::vector<projection_candidate> make_projection_candidates(
    cached_run_shape const& shape,
    geo::latlng const& position,
    std::optional<std::size_t> const constrained_stop_idx,
    bool const stopped_at) {
  auto candidates = std::vector<projection_candidate>{};
  candidates.reserve(shape.points_.size() + shape.stop_point_indices_.size());
  if (stopped_at ||
      (constrained_stop_idx.has_value() && *constrained_stop_idx == 0U)) {
    auto const stop = *constrained_stop_idx;
    auto const point = shape.stop_point_indices_[stop];
    candidates.push_back(
        {.lateral_error_ = geo::distance(position, shape.points_[point]),
         .distance_along_ = shape.stop_distances_[stop],
         .next_stop_idx_ = stop});
    return candidates;
  }

  for (auto section = std::size_t{0U};
       section + 1U < shape.stop_point_indices_.size(); ++section) {
    auto const next_stop = section + 1U;
    if (constrained_stop_idx.has_value() &&
        next_stop != *constrained_stop_idx) {
      continue;
    }
    auto const from = shape.stop_point_indices_[section];
    auto const to = shape.stop_point_indices_[next_stop];
    if (from > to) {
      continue;
    }
    if (from == to) {
      candidates.push_back(
          {.lateral_error_ = geo::distance(position, shape.points_[from]),
           .distance_along_ = shape.stop_distances_[next_stop],
           .next_stop_idx_ = next_stop});
      continue;
    }
    for (auto segment = from; segment != to; ++segment) {
      if (shape.points_[segment] == shape.points_[segment + 1U]) {
        candidates.push_back(
            {.lateral_error_ = geo::distance(position, shape.points_[segment]),
             .distance_along_ = shape.point_distances_[segment],
             .next_stop_idx_ = next_stop});
        continue;
      }
      auto const segment_shape = std::span{shape.points_}.subspan(segment, 2U);
      auto const projected = geo::distance_to_polyline(position, segment_shape);
      candidates.push_back(
          {.lateral_error_ = projected.distance_to_polyline_,
           .distance_along_ =
               shape.point_distances_[segment] +
               geo::distance(shape.points_[segment], projected.best_),
           .next_stop_idx_ = next_stop});
    }
  }
  return candidates;
}

candidate_selection select_candidate(
    std::vector<projection_candidate> candidates,
    std::optional<trip_progress> const& prior) {
  constexpr auto kEquivalentProgressMeters = 25.0;
  if (prior.has_value()) {
    std::erase_if(candidates, [&](projection_candidate const& candidate) {
      return candidate.distance_along_ + kEquivalentProgressMeters <
             prior->distance_along_shape_m_;
    });
    if (candidates.empty()) {
      return {.status_ = trip_progress_projection_status::kImplausible,
              .candidate_ = std::nullopt};
    }
  }

  auto const best_lateral =
      std::ranges::min(candidates, {}, &projection_candidate::lateral_error_)
          .lateral_error_;
  constexpr auto kMaxLateralErrorMeters = 100.0;
  if (best_lateral > kMaxLateralErrorMeters) {
    return {.status_ = trip_progress_projection_status::kOffShape,
            .candidate_ = std::nullopt};
  }

  constexpr auto kEquivalentLateralErrorMeters = 5.0;
  std::erase_if(candidates, [&](projection_candidate const& candidate) {
    return candidate.lateral_error_ >
           best_lateral + kEquivalentLateralErrorMeters;
  });

  std::ranges::sort(candidates, {}, &projection_candidate::distance_along_);
  if (candidates.size() > 1U &&
      candidates.back().distance_along_ - candidates.front().distance_along_ >
          kEquivalentProgressMeters) {
    return {.status_ = trip_progress_projection_status::kAmbiguous,
            .candidate_ = std::nullopt};
  }
  constexpr auto kProjectionTieToleranceMeters = 1e-3;
  for (auto i = std::size_t{0U}; i != candidates.size(); ++i) {
    for (auto j = i + 1U; j != candidates.size(); ++j) {
      auto const equal_quality = std::abs(candidates[i].lateral_error_ -
                                          candidates[j].lateral_error_) <=
                                 kProjectionTieToleranceMeters;
      auto const conflicting_progress =
          std::abs(candidates[i].distance_along_ -
                   candidates[j].distance_along_) >
          kProjectionTieToleranceMeters;
      if (equal_quality &&
          (conflicting_progress ||
           candidates[i].next_stop_idx_ != candidates[j].next_stop_idx_)) {
        return {.status_ = trip_progress_projection_status::kAmbiguous,
                .candidate_ = std::nullopt};
      }
    }
  }
  std::ranges::sort(candidates, {}, &projection_candidate::lateral_error_);
  return {.status_ = trip_progress_projection_status::kProjected,
          .candidate_ = candidates.front()};
}

}  // namespace

struct trip_progress_projector::impl {
  static constexpr auto kMaxCachedRunShapes = 1024U;

  struct cache_entry {
    cached_run_shape shape_;
    std::list<cache_key>::iterator lru_it_;
  };

  explicit impl(n::shapes_storage const& shapes) : shapes_{shapes} {}

  cached_run_shape const* get_shape(n::rt::frun const& fr) {
    auto const segment = get_trip_segment(fr);
    if (!segment.has_value()) {
      return nullptr;
    }
    auto const key = cache_key{segment->trip_, fr.stop_range_.from_,
                               fr.stop_range_.to_};
    if (auto const it = cache_.find(key); it != end(cache_)) {
      lru_.splice(end(lru_), lru_, it->second.lru_it_);
      return &it->second.shape_;
    }

    auto shape = make_cached_shape(fr, shapes_, *segment);
    if (!shape.has_value()) {
      return nullptr;
    }
    if (cache_.size() == kMaxCachedRunShapes) {
      cache_.erase(lru_.front());
      lru_.pop_front();
    }
    lru_.push_back(key);
    auto const it =
        cache_
            .emplace(key, cache_entry{std::move(*shape), std::prev(end(lru_))})
            .first;
    return &it->second.shape_;
  }

  n::shapes_storage const& shapes_;
  std::list<cache_key> lru_;
  std::map<cache_key, cache_entry> cache_;
};

trip_progress_projector::trip_progress_projector(
    n::shapes_storage const& shapes)
    : impl_{std::make_unique<impl>(shapes)} {}

trip_progress_projector::~trip_progress_projector() = default;

trip_progress_projector::trip_progress_projector(
    trip_progress_projector&&) noexcept = default;

trip_progress_projector& trip_progress_projector::operator=(
    trip_progress_projector&&) noexcept = default;

trip_progress_projection trip_progress_projector::project(
    n::rt::frun const& fr,
    geo::latlng const& position,
    std::optional<trip_progress> const& prior,
    std::optional<vehicle_position_progress_constraint> const& vp_constraint) {
  if (!fr.is_scheduled()) {
    return {};
  }
  auto const* shape = impl_->get_shape(fr);
  if (shape == nullptr) {
    return {};
  }

  auto constrained_stop_idx = std::optional<std::size_t>{};
  if (vp_constraint.has_value()) {
    constrained_stop_idx = get_constrained_stop_idx(*shape, *vp_constraint);
    if (!constrained_stop_idx.has_value()) {
      return {.status_ = trip_progress_projection_status::kImplausible,
              .progress_ = std::nullopt};
    }
  }

  auto candidates = make_projection_candidates(
      *shape, position, constrained_stop_idx,
      vp_constraint.has_value() &&
          vp_constraint->status_ == vehicle_position_stop_status::kStoppedAt);
  if (candidates.empty()) {
    return {.status_ = vp_constraint.has_value()
                           ? trip_progress_projection_status::kImplausible
                           : trip_progress_projection_status::kMissingShape,
            .progress_ = std::nullopt};
  }

  auto selection = select_candidate(std::move(candidates), prior);
  if (selection.status_ == trip_progress_projection_status::kOffShape &&
      vp_constraint.has_value() &&
      vp_constraint->status_ != vehicle_position_stop_status::kStoppedAt &&
      *constrained_stop_idx + 1U < shape->stop_point_indices_.size()) {
    // Some producers use current_stop_sequence for the last passed stop
    // instead of the next stop. Stay local: only try the following section
    // after the standards-compliant section is geometrically impossible.
    selection = select_candidate(
        make_projection_candidates(*shape, position,
                                   *constrained_stop_idx + 1U, false),
        prior);
  }
  if (prior.has_value() && vp_constraint.has_value() &&
      vp_constraint->status_ != vehicle_position_stop_status::kStoppedAt &&
      shape->stop_point_indices_.front() !=
          shape->stop_point_indices_.back()) {
    // VehiclePosition current_stop_sequence is useful for disambiguating
    // loops, but some feeds lag it by several stops. Once geometry has already
    // established forward progress, treat the sequence as a hint when the
    // full-shape projection is materially closer. Ambiguous geometry still
    // fails closed in select_candidate.
    auto const unconstrained = select_candidate(
        make_projection_candidates(*shape, position, std::nullopt, false),
        prior);
    constexpr auto kConstraintOverrideImprovementMeters = 5.0;
    if (unconstrained.candidate_.has_value() &&
        (!selection.candidate_.has_value() ||
         unconstrained.candidate_->lateral_error_ +
                 kConstraintOverrideImprovementMeters <
             selection.candidate_->lateral_error_)) {
      selection = unconstrained;
    }
  }
  if (!selection.candidate_.has_value()) {
    return {.status_ = selection.status_, .progress_ = std::nullopt};
  }
  if (!vp_constraint.has_value() &&
      shape->stop_point_indices_.front() == shape->stop_point_indices_.back()) {
    return {.status_ = trip_progress_projection_status::kAmbiguous,
            .progress_ = std::nullopt};
  }
  auto const& candidate = *selection.candidate_;
  auto monotonicity = trip_progress_monotonicity::kNoPrior;
  if (prior.has_value()) {
    auto const delta =
        candidate.distance_along_ - prior->distance_along_shape_m_;
    monotonicity = delta > 1.0    ? trip_progress_monotonicity::kForward
                   : delta < -1.0 ? trip_progress_monotonicity::kMinorRegression
                                  : trip_progress_monotonicity::kStationary;
  }
  return {.status_ = trip_progress_projection_status::kProjected,
          .progress_ = trip_progress{
              .distance_along_shape_m_ = candidate.distance_along_,
              .lateral_error_m_ = candidate.lateral_error_,
              .next_static_stop_sequence_ =
                  shape->static_stop_sequences_[candidate.next_stop_idx_],
              .distance_to_next_stop_m_ = std::max(
                  0.0, shape->stop_distances_[candidate.next_stop_idx_] -
                           candidate.distance_along_),
              .monotonicity_ = monotonicity}};
}

std::optional<std::vector<trip_progress_stop>>
trip_progress_projector::stop_timeline(n::rt::frun const& fr) {
  if (!fr.is_scheduled()) {
    return std::nullopt;
  }
  auto const* shape = impl_->get_shape(fr);
  if (shape == nullptr ||
      shape->stop_distances_.size() != shape->static_stop_sequences_.size()) {
    return std::nullopt;
  }
  auto result = std::vector<trip_progress_stop>{};
  auto const stop_count =
      static_cast<n::stop_idx_t>(shape->stop_distances_.size());
  result.reserve(stop_count);
  for (auto i = n::stop_idx_t{0U}; i != stop_count; ++i) {
    auto const arrival_event =
        i == 0U ? n::event_type::kDep : n::event_type::kArr;
    auto const departure_event =
        i + 1U == stop_count ? n::event_type::kArr : n::event_type::kDep;
    result.push_back({.static_stop_sequence_ = shape->static_stop_sequences_[i],
                      .distance_along_shape_m_ = shape->stop_distances_[i],
                      .scheduled_arrival_time_ =
                          to_seconds(fr[i].scheduled_time(arrival_event)),
                      .scheduled_departure_time_ =
                          to_seconds(fr[i].scheduled_time(departure_event))});
  }
  return result;
}

}  // namespace motis
