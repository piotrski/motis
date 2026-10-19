#include "motis/rt/passed_stops.h"

#include <algorithm>
#include <chrono>

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/timetable.h"

#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {

namespace {

using event_key = passed_stop_store::event_key;
using trip_key = passed_stop_store::trip_key;

n::unixtime_t to_unixtime(std::int64_t const seconds) {
  return std::chrono::floor<n::i32_minutes>(
      std::chrono::sys_seconds{std::chrono::seconds{seconds}});
}

vehicle_prediction_event_type to_prediction_event(n::event_type const ev) {
  return ev == n::event_type::kArr ? vehicle_prediction_event_type::kArrival
                                   : vehicle_prediction_event_type::kDeparture;
}

struct resolved_position {
  n::rt::run run_;
  std::vector<unsigned> sequences_;
  n::stop_idx_t first_covered_;
};

std::optional<resolved_position> resolve_position(
    passed_stop_update_context const& ctx,
    n::rt_timetable const& rtt,
    trip_update_position const& position) {
  auto const [run, trip] = n::rt::gtfsrt_resolve_run(
      ctx.today_, ctx.tt_, &rtt, position.src_, position.trip_);
  if (!run.valid() || !run.is_scheduled() || !run.is_rt() ||
      trip == n::trip_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const encoded = ctx.tt_.trip_stop_seq_numbers_[trip];
  auto const sequence_range = n::loader::gtfs::stop_seq_number_range{
      {encoded.data(), encoded.size()},
      static_cast<n::stop_idx_t>(run.stop_range_.size())};
  auto sequences = std::vector<unsigned>{};
  for (auto const sequence : sequence_range) {
    sequences.push_back(static_cast<unsigned>(sequence));
  }

  // Same matching as nigiri's update_run: stop_sequence where the feed has
  // it, the scheduled stop_id otherwise (ZTP trams send stop_id only).
  auto const location_seq =
      ctx.tt_.route_location_seq_[ctx.tt_.transport_route_[run.t_.t_idx_]];
  for (auto i = n::stop_idx_t{0U}; i != sequences.size(); ++i) {
    auto const stop_idx = static_cast<n::stop_idx_t>(run.stop_range_.from_ + i);
    auto const location = n::stop{location_seq[stop_idx]}.location_idx();
    if ((position.first_stop_sequence_.has_value() &&
         *position.first_stop_sequence_ == sequences[i]) ||
        (position.first_stop_id_.has_value() &&
         *position.first_stop_id_ == ctx.tt_.locations_.ids_[location].view())) {
      return resolved_position{.run_ = run,
                               .sequences_ = std::move(sequences),
                               .first_covered_ = stop_idx};
    }
  }
  return std::nullopt;
}

// Last time the previous cycle published for an event the vehicle has now
// passed, with the source it was shown with: its selected prediction to the
// second where one exists, the (minute) timetable time otherwise. GPS-selected
// events prefer the interpolated passage the GPS history observed.
std::optional<passed_stop_event> freeze(passed_stop_update_context const& ctx,
                                        resolved_position const& position,
                                        n::stop_idx_t const stop_idx,
                                        n::event_type const ev,
                                        std::string_view const trip_id,
                                        std::int64_t const omitted_at) {
  auto const& run = position.run_;
  auto const previous_rt = ctx.previous_rtt_->resolve_rt(run.t_);
  if (previous_rt == n::rt_transport_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const sequence = position.sequences_[stop_idx - run.stop_range_.from_];
  auto const scheduled = to_seconds(ctx.tt_.event_time(run.t_, stop_idx, ev));
  auto time = to_seconds(
      ctx.previous_rtt_->unix_event_time(previous_rt, stop_idx, ev));
  auto source = vehicle_prediction_source::kProvider;

  auto const* diagnostic =
      ctx.previous_diagnostics_ == nullptr
          ? nullptr
          : ctx.previous_diagnostics_->find_event(run.t_, trip_id, sequence,
                                                  scheduled,
                                                  to_prediction_event(ev));
  if (diagnostic != nullptr) {
    time = diagnostic->effective_.predicted_timestamp_seconds_;
  }
  if (diagnostic != nullptr &&
      diagnostic->effective_.source_ == vehicle_prediction_source::kGps) {
    source = vehicle_prediction_source::kGps;
    for (auto const& candidate : ctx.candidates_) {
      if (candidate.batch_.transport_ != run.t_ ||
          candidate.trip_stop_range_ != run.stop_range_) {
        continue;
      }
      auto const passage = std::ranges::find(
          candidate.batch_.observed_passages_, sequence,
          &observed_stop_passage::static_stop_sequence_);
      if (passage != end(candidate.batch_.observed_passages_)) {
        time = passage->observed_timestamp_seconds_;
      }
    }
  }
  return passed_stop_event{.time_seconds_ = std::min(time, omitted_at),
                           .source_ = source,
                           .frozen_at_seconds_ = ctx.now_seconds_};
}

// GPS time the overlay is about to render for the vehicle's next event, so
// passed events can be kept before it and the overlay's chronology check
// still accepts the transport.
std::optional<n::unixtime_t> overlay_time(passed_stop_update_context const& ctx,
                                          resolved_position const& position,
                                          n::stop_idx_t const stop_idx,
                                          n::event_type const ev) {
  auto const sequence =
      position.sequences_[stop_idx - position.run_.stop_range_.from_];
  auto result = std::optional<n::unixtime_t>{};
  for (auto const& overlay : ctx.overlays_) {
    if (overlay.transport_ != position.run_.t_) {
      continue;
    }
    for (auto const& prediction : overlay.predictions_) {
      if (prediction.static_stop_sequence_ == sequence &&
          prediction.event_type_ == to_prediction_event(ev)) {
        auto const t = to_unixtime(prediction.predicted_timestamp_seconds_);
        result = result.has_value() ? std::min(*result, t) : t;
      }
    }
  }
  return result;
}

void apply(passed_stop_update_context const& ctx,
           n::rt_timetable& rtt,
           resolved_position const& position,
           passed_stop_store const& store) {
  auto const& run = position.run_;
  auto const rt_transport = rtt.resolve_rt(run.t_);
  auto const write = [&](n::stop_idx_t const stop_idx, n::event_type const ev,
                         n::unixtime_t const time) {
    if (rtt.unix_event_time(rt_transport, stop_idx, ev) == time) {
      return;
    }
    rtt.update_time(rt_transport, stop_idx, ev, time);
    rtt.dispatch_delay(run, stop_idx, ev,
                       time - ctx.tt_.event_time(run.t_, stop_idx, ev));
  };

  // Walk back from the vehicle's next event over the whole transport: each
  // passed event takes its frozen time, and no event may run later than the
  // one after it.
  auto const next = position.first_covered_;
  auto bound = rtt.unix_event_time(rt_transport, next, n::event_type::kArr);
  if (auto const gps = overlay_time(ctx, position, next, n::event_type::kArr);
      gps.has_value()) {
    bound = std::min(bound, *gps);
  }
  for (auto stop_idx = static_cast<int>(next) - 1; stop_idx >= 0; --stop_idx) {
    auto const idx = static_cast<n::stop_idx_t>(stop_idx);
    for (auto const ev : {n::event_type::kDep, n::event_type::kArr}) {
      if (ev == n::event_type::kArr && idx == 0U) {
        continue;
      }
      auto const* frozen = store.find(run.t_, idx, ev);
      auto const time =
          std::min(frozen != nullptr ? to_unixtime(frozen->time_seconds_)
                                     : rtt.unix_event_time(rt_transport, idx, ev),
                   bound);
      write(idx, ev, time);
      bound = time;
    }
  }
}

}  // namespace

std::vector<trip_update_position> trip_update_positions(
    n::source_idx_t const src, transit_realtime::FeedMessage const& msg) {
  auto positions = std::vector<trip_update_position>{};
  auto const header_timestamp =
      msg.has_header() && msg.header().timestamp() != 0U
          ? std::optional{static_cast<std::int64_t>(msg.header().timestamp())}
          : std::nullopt;
  for (auto const& entity : msg.entity()) {
    if (!entity.has_trip_update() ||
        entity.trip_update().stop_time_update_size() == 0 ||
        (entity.has_is_deleted() && entity.is_deleted())) {
      continue;
    }
    auto const& update = entity.trip_update();
    auto const relationship = update.trip().schedule_relationship();
    if (relationship !=
        transit_realtime::TripDescriptor_ScheduleRelationship_SCHEDULED) {
      continue;
    }
    auto const& first = update.stop_time_update(0);
    if (!first.has_stop_sequence() && !first.has_stop_id()) {
      continue;
    }
    positions.push_back(
        {.src_ = src,
         .trip_ = update.trip(),
         .first_stop_sequence_ = first.has_stop_sequence()
                                     ? std::optional{first.stop_sequence()}
                                     : std::nullopt,
         .first_stop_id_ = first.has_stop_id()
                               ? std::optional{first.stop_id()}
                               : std::nullopt,
         .reported_at_seconds_ =
             update.timestamp() != 0U
                 ? std::optional{static_cast<std::int64_t>(update.timestamp())}
                 : header_timestamp});
  }
  return positions;
}

passed_stop_event const* passed_stop_store::find(
    n::transport const transport,
    n::stop_idx_t const stop_idx,
    n::event_type const event_type) const {
  auto const it = events_.find(event_key{transport, stop_idx, event_type});
  return it == end(events_) ? nullptr : &it->second;
}

passed_stop_store update_passed_stops(
    passed_stop_update_context const& ctx,
    n::rt_timetable& rtt,
    std::span<trip_update_position const> const positions) {
  auto store = passed_stop_store{};
  auto resolved = std::vector<resolved_position>{};
  for (auto const& position : positions) {
    auto r = resolve_position(ctx, rtt, position);
    if (!r.has_value()) {
      continue;
    }
    auto const& run = r->run_;
    auto const key = trip_key{run.t_, run.stop_range_.from_};
    store.first_covered_.insert_or_assign(key, r->first_covered_);

    auto const previous_first =
        ctx.previous_ == nullptr
            ? std::nullopt
            : [&]() -> std::optional<n::stop_idx_t> {
          auto const it = ctx.previous_->first_covered_.find(key);
          return it == end(ctx.previous_->first_covered_)
                     ? std::nullopt
                     : std::optional{it->second};
        }();
    auto const omitted_at = std::min(
        position.reported_at_seconds_.value_or(ctx.now_seconds_),
        ctx.now_seconds_);
    auto const trip_id = ctx.tags_.id(
        ctx.tt_, n::rt::frun{ctx.tt_, &rtt, run}[0], n::event_type::kDep);
    for (auto idx = run.stop_range_.from_; idx < r->first_covered_; ++idx) {
      for (auto const ev : {n::event_type::kArr, n::event_type::kDep}) {
        if (ev == n::event_type::kArr && idx == run.stop_range_.from_) {
          continue;  // belongs to the previous trip of the block
        }
        auto const k = event_key{run.t_, idx, ev};
        if (auto const* kept =
                ctx.previous_ == nullptr ? nullptr
                                         : ctx.previous_->find(run.t_, idx, ev);
            kept != nullptr) {
          store.events_.insert_or_assign(k, *kept);
          continue;
        }
        // Only stops the feed still described last cycle get frozen: their
        // last published time is a real prediction, not the schedule.
        if (!previous_first.has_value() || idx < *previous_first ||
            ctx.previous_rtt_ == nullptr) {
          continue;
        }
        if (auto const frozen = freeze(ctx, *r, idx, ev, trip_id, omitted_at);
            frozen.has_value()) {
          store.events_.insert_or_assign(k, *frozen);
        }
      }
    }
    resolved.emplace_back(std::move(*r));
  }
  for (auto const& position : resolved) {
    if (position.first_covered_ != position.run_.stop_range_.from_) {
      apply(ctx, rtt, position, store);
    }
  }
  return store;
}

}  // namespace motis
