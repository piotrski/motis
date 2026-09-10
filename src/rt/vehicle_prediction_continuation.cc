#include "motis/rt/vehicle_prediction_continuation.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <span>
#include <utility>

#include "utl/enumerate.h"

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/frun.h"
#include "nigiri/timetable.h"

#include "motis/rt/vehicle_prediction_diagnostics.h"
#include "motis/rt/vehicle_prediction_store.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {
namespace {

struct trip_segment {
  n::trip_idx_t trip_{n::trip_idx_t::invalid()};
  n::interval<n::stop_idx_t> range_{};
};

using continuation_key = std::pair<n::transport, std::string>;
using continuation_entries =
    std::vector<vehicle_prediction_diagnostic_entry const*>;
using continuation_groups = std::map<continuation_key, continuation_entries>;

std::vector<trip_segment> trip_segments(n::rt::frun const& run) {
  auto result = std::vector<trip_segment>{};
  run.for_each_trip(
      [&](n::trip_idx_t const trip, n::interval<n::stop_idx_t> const range) {
        result.push_back({trip, range});
      });
  return result;
}

std::optional<std::pair<trip_segment, trip_segment>> adjacent_segments(
    n::rt::frun const& full, n::rt::frun const& current) {
  auto const segments = trip_segments(full);
  auto const current_trip = current[0U].get_trip_idx(n::event_type::kDep);
  auto const matches_current = [&](trip_segment const& segment) {
    return segment.trip_ == current_trip &&
           segment.range_ == current.stop_range_;
  };
  auto const match = std::ranges::find_if(segments, matches_current);
  if (match == end(segments)) {
    return std::nullopt;
  }
  auto const adjacent = std::next(match);
  if (adjacent == end(segments) ||
      std::ranges::find_if(adjacent, end(segments), matches_current) !=
          end(segments)) {
    return std::nullopt;
  }
  return std::pair{*match, *adjacent};
}

bool is_shared_adjacent_terminal(n::rt::frun const& full,
                                 trip_segment const& current,
                                 trip_segment const& next) {
  if (current.range_.to_ == 0U ||
      next.range_.from_ + 1U != current.range_.to_ ||
      next.range_.size() < n::stop_idx_t{2U} ||
      next.trip_ == n::trip_idx_t::invalid()) {
    return false;
  }
  auto const terminal = static_cast<n::stop_idx_t>(current.range_.to_ - 1U);
  return full[terminal].get_location_idx() ==
         full[next.range_.from_].get_location_idx();
}

std::optional<std::int64_t> predicted_terminal_arrival(
    n::rt::frun const& current, vehicle_prediction_batch const& prediction) {
  auto const scheduled =
      to_seconds(current[current.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto const it = std::ranges::find_if(
      prediction.predictions_, [&](vehicle_stop_prediction const& event) {
        return event.event_type_ == vehicle_prediction_event_type::kArrival &&
               event.scheduled_timestamp_seconds_ == scheduled;
      });
  return it == end(prediction.predictions_)
             ? std::nullopt
             : std::optional{it->predicted_timestamp_seconds_};
}

std::vector<vehicle_stop_prediction> next_leg_predictions(
    n::timetable const& tt,
    n::rt::frun const& next_run,
    trip_segment const& next,
    std::int64_t const delay,
    std::int64_t const reference) {
  auto const encoded = tt.trip_stop_seq_numbers_[next.trip_];
  auto const sequence_range = n::loader::gtfs::stop_seq_number_range{
      {encoded.data(), encoded.size()},
      static_cast<n::stop_idx_t>(next.range_.size())};
  auto const sequences =
      std::vector<n::stop_idx_t>{begin(sequence_range), end(sequence_range)};
  auto predictions = std::vector<vehicle_stop_prediction>{};
  predictions.reserve(sequences.size() * 2U - 2U);
  for (auto const [idx, stop_sequence] : utl::enumerate(sequences)) {
    auto const stop_idx = static_cast<n::stop_idx_t>(idx);
    auto const append = [&](vehicle_prediction_event_type const event_type,
                            n::event_type const nigiri_event) {
      auto const scheduled =
          to_seconds(next_run[stop_idx].scheduled_time(nigiri_event));
      predictions.push_back(
          {.static_stop_sequence_ = static_cast<unsigned>(stop_sequence),
           .event_type_ = event_type,
           .scheduled_timestamp_seconds_ = scheduled,
           .predicted_timestamp_seconds_ = scheduled + delay,
           .delay_seconds_ = delay,
           .horizon_seconds_ = scheduled + delay - reference});
    };
    if (idx != 0U) {
      append(vehicle_prediction_event_type::kArrival, n::event_type::kArr);
    }
    if (idx + 1U != sequences.size()) {
      append(vehicle_prediction_event_type::kDeparture, n::event_type::kDep);
    }
  }
  return predictions;
}

incoming_leg_prediction_provenance incoming_provenance(
    n::rt::frun const& current,
    std::string_view const feed,
    std::string_view const current_trip_id,
    std::string const& next_trip_id,
    std::int64_t const expected_arrival,
    std::int64_t const scheduled_departure,
    std::int64_t const delay,
    std::int64_t const reference) {
  auto const incoming = current[0U];
  return {.feed_ = std::string{feed},
          .incoming_trip_id_ = std::string{current_trip_id},
          .next_trip_id_ = next_trip_id,
          .route_id_ = std::string{incoming.get_route_id(n::event_type::kDep)},
          .route_short_name_ = std::string{incoming.route_short_name(
              n::event_type::kDep, n::lang_t{})},
          .headsign_ =
              std::string{incoming.direction(n::lang_t{}, n::event_type::kDep)},
          .mode_ = incoming.get_clasz(n::event_type::kDep),
          .expected_terminal_arrival_seconds_ = expected_arrival,
          .next_scheduled_departure_seconds_ = scheduled_departure,
          .propagated_delay_seconds_ = delay,
          .reference_timestamp_seconds_ = reference};
}

continuation_groups collect_fresh_continuations(
    vehicle_prediction_diagnostics_store const& previous,
    std::int64_t const now,
    std::int64_t const max_age) {
  auto grouped = continuation_groups{};
  for (auto const& entry : previous.entries_) {
    if (entry.gps_context_ != vehicle_prediction_context::kIncomingBlockLeg ||
        !entry.incoming_leg_provenance_.has_value() ||
        !entry.trip_stop_range_.has_value() || !entry.gps_.has_value() ||
        !entry.gps_->reference_timestamp_seconds_.has_value() ||
        now < *entry.gps_->reference_timestamp_seconds_ ||
        now - *entry.gps_->reference_timestamp_seconds_ > max_age) {
      continue;
    }
    grouped[{entry.transport_, entry.trip_id_}].push_back(&entry);
  }
  return grouped;
}

bool has_eligible_current_prediction(
    std::span<vehicle_prediction_cycle_result const> const current,
    continuation_key const& key) {
  auto const& [transport, trip_id] = key;
  return std::ranges::any_of(
      current, [&](vehicle_prediction_cycle_result const& candidate) {
        return candidate.batch_.transport_ == transport &&
               candidate.trip_id_ == trip_id && candidate.batch_.eligible();
      });
}

bool preferred_current_prediction(vehicle_prediction_cycle_result const& a,
                                  vehicle_prediction_cycle_result const& b) {
  auto const direct = [](vehicle_prediction_cycle_result const& candidate) {
    return candidate.context_ == vehicle_prediction_context::kDirect;
  };
  if (direct(a) != direct(b)) {
    return direct(a);
  }
  return a.batch_.candidate_reference_timestamp_seconds_.value_or(0) >
         b.batch_.candidate_reference_timestamp_seconds_.value_or(0);
}

std::vector<vehicle_prediction_cycle_result> canonicalize_current_predictions(
    std::vector<vehicle_prediction_cycle_result> current) {
  auto winners =
      std::map<continuation_key, vehicle_prediction_cycle_result const*>{};
  for (auto const& candidate : current) {
    if (!candidate.batch_.eligible()) {
      continue;
    }
    auto const key =
        continuation_key{candidate.batch_.transport_, candidate.trip_id_};
    auto [it, inserted] = winners.try_emplace(key, &candidate);
    if (!inserted && preferred_current_prediction(candidate, *it->second)) {
      it->second = &candidate;
    }
  }
  auto canonical = std::vector<vehicle_prediction_cycle_result>{};
  canonical.reserve(current.size());
  for (auto& candidate : current) {
    auto const key =
        continuation_key{candidate.batch_.transport_, candidate.trip_id_};
    auto const winner = winners.find(key);
    if (winner == end(winners) || winner->second == &candidate) {
      canonical.emplace_back(std::move(candidate));
    }
  }
  return canonical;
}

bool consistent_continuation(continuation_entries const& entries) {
  auto const& first = *entries.front();
  auto const reference = first.gps_->reference_timestamp_seconds_;
  return std::ranges::all_of(entries, [&](auto const* entry) {
    return entry->trip_stop_range_ == first.trip_stop_range_ &&
           entry->incoming_leg_provenance_ == first.incoming_leg_provenance_ &&
           entry->gps_->reference_timestamp_seconds_ == reference;
  });
}

vehicle_prediction_cycle_result rebuild_continuation(
    continuation_key const& key,
    continuation_entries const& entries,
    std::int64_t const now) {
  auto const& [transport, trip_id] = key;
  auto const& first = *entries.front();
  auto const& provenance = *first.incoming_leg_provenance_;
  auto const reference = *first.gps_->reference_timestamp_seconds_;
  auto retained = vehicle_prediction_cycle_result{
      .feed_ = provenance.feed_,
      .trip_id_ = trip_id,
      .mode_ = provenance.mode_,
      .trip_stop_range_ = first.trip_stop_range_,
      .context_ = vehicle_prediction_context::kIncomingBlockLeg,
      .incoming_leg_provenance_ = provenance,
      .latest_vehicle_observation_timestamp_seconds_ = reference};
  retained.batch_.transport_ = transport;
  retained.batch_.candidate_reference_timestamp_seconds_ = reference;
  retained.batch_.confidence_ = vehicle_prediction_confidence{
      .score_ = first.gps_->confidence_.value_or(0.0)};
  retained.batch_.predictions_.reserve(entries.size());
  std::ranges::transform(
      entries, std::back_inserter(retained.batch_.predictions_),
      [&](auto const* entry) {
        return vehicle_stop_prediction{
            .static_stop_sequence_ = entry->static_stop_sequence_,
            .event_type_ = entry->event_type_,
            .scheduled_timestamp_seconds_ = entry->scheduled_timestamp_seconds_,
            .predicted_timestamp_seconds_ =
                entry->gps_->predicted_timestamp_seconds_,
            .delay_seconds_ = entry->gps_->delay_seconds_,
            .horizon_seconds_ =
                entry->gps_->predicted_timestamp_seconds_ - now};
      });
  return retained;
}

}  // namespace

std::optional<vehicle_prediction_continuation>
derive_vehicle_prediction_continuation(
    n::timetable const& tt,
    tag_lookup const& tags,
    n::rt::frun const& current_run,
    std::string_view const feed,
    std::string_view const current_trip_id,
    vehicle_prediction_batch const& prediction) {
  if (!current_run.is_scheduled() || !prediction.eligible() ||
      prediction.transport_ != current_run.t_ ||
      !prediction.candidate_reference_timestamp_seconds_.has_value() ||
      !prediction.confidence_.has_value() || current_run.stop_range_.empty()) {
    return std::nullopt;
  }

  auto const full = n::rt::frun::from_t(tt, current_run.rtt_, current_run.t_);
  auto const adjacent = adjacent_segments(full, current_run);
  if (!adjacent.has_value()) {
    return std::nullopt;
  }
  auto const& [current, next] = *adjacent;
  if (!is_shared_adjacent_terminal(full, current, next)) {
    return std::nullopt;
  }

  auto const expected_arrival =
      predicted_terminal_arrival(current_run, prediction);
  if (!expected_arrival.has_value()) {
    return std::nullopt;
  }
  auto next_run = full;
  next_run.stop_range_ = next.range_;
  auto const scheduled_departure =
      to_seconds(next_run[0].scheduled_time(n::event_type::kDep));
  auto const delay =
      std::max<std::int64_t>(0, *expected_arrival - scheduled_departure);
  auto const reference = *prediction.candidate_reference_timestamp_seconds_;
  auto const first = next_run[0];
  auto const next_trip_id = tags.id(tt, first, n::event_type::kDep);
  return vehicle_prediction_continuation{
      .transport_ = current_run.t_,
      .trip_stop_range_ = next.range_,
      .trip_id_ = next_trip_id,
      .mode_ = first.get_clasz(n::event_type::kDep),
      .reference_timestamp_seconds_ = reference,
      .confidence_ = prediction.confidence_->score_,
      .predictions_ =
          next_leg_predictions(tt, next_run, next, delay, reference),
      .provenance_ = incoming_provenance(
          current_run, feed, current_trip_id, next_trip_id, *expected_arrival,
          scheduled_departure, delay, reference)};
}

std::vector<vehicle_prediction_cycle_result>
merge_retained_vehicle_prediction_continuations(
    vehicle_prediction_diagnostics_store const* const previous,
    std::vector<vehicle_prediction_cycle_result> current,
    std::int64_t const now_seconds,
    std::int64_t const max_age_seconds) {
  if (previous == nullptr) {
    return canonicalize_current_predictions(std::move(current));
  }
  current = canonicalize_current_predictions(std::move(current));
  for (auto const& [key, entries] :
       collect_fresh_continuations(*previous, now_seconds, max_age_seconds)) {
    if (entries.empty() || has_eligible_current_prediction(current, key) ||
        !consistent_continuation(entries)) {
      continue;
    }
    auto const& [transport, trip_id] = key;
    std::erase_if(current, [&](vehicle_prediction_cycle_result const& x) {
      return x.batch_.transport_ == transport && x.trip_id_ == trip_id;
    });
    current.emplace_back(rebuild_continuation(key, entries, now_seconds));
  }
  return current;
}

}  // namespace motis
