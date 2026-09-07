#include "motis/rt/vehicle_prediction_overlay.h"

#include <algorithm>
#include <map>
#include <optional>
#include <tuple>
#include <utility>

#include "utl/enumerate.h"

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/timetable.h"

#include "motis/rt/vehicle_prediction_effective.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {
namespace {

struct event_update {
  n::stop_idx_t stop_idx_{};
  unsigned static_stop_sequence_{};
  n::event_type event_type_{n::event_type::kArr};
  n::unixtime_t time_{};
  n::duration_t delay_{};
};

struct prepared_trip {
  selected_vehicle_prediction_trip const* trip_{};
  n::interval<n::stop_idx_t> range_{};
  std::vector<event_update> updates_;
};

std::optional<std::int64_t> previous_delay(
    vehicle_prediction_overlay_state const& state,
    selected_vehicle_prediction_trip const& trip,
    vehicle_stop_prediction const& prediction) {
  auto const it =
      std::ranges::find_if(state.rendered_events_, [&](auto const& event) {
        return event.transport_ == trip.transport_ &&
               event.trip_stop_range_ == trip.trip_stop_range_ &&
               event.static_stop_sequence_ ==
                   prediction.static_stop_sequence_ &&
               event.event_type_ == prediction.event_type_;
      });
  return it == end(state.rendered_events_) ? std::nullopt
                                           : std::optional{it->delay_minutes_};
}

void remember_delay(vehicle_prediction_overlay_state& state,
                    selected_vehicle_prediction_trip const& trip,
                    vehicle_stop_prediction const& prediction,
                    std::int64_t const minutes,
                    std::optional<std::int64_t> const effective_timestamp) {
  auto const it =
      std::ranges::find_if(state.rendered_events_, [&](auto const& event) {
        return event.transport_ == trip.transport_ &&
               event.trip_stop_range_ == trip.trip_stop_range_ &&
               event.static_stop_sequence_ ==
                   prediction.static_stop_sequence_ &&
               event.event_type_ == prediction.event_type_;
      });
  if (it == end(state.rendered_events_)) {
    state.rendered_events_.push_back(
        {.transport_ = trip.transport_,
         .trip_stop_range_ = trip.trip_stop_range_,
         .static_stop_sequence_ = prediction.static_stop_sequence_,
         .event_type_ = prediction.event_type_,
         .delay_minutes_ = minutes,
         .effective_timestamp_seconds_ = effective_timestamp});
  } else {
    it->delay_minutes_ = minutes;
    it->effective_timestamp_seconds_ = effective_timestamp;
  }
}

std::optional<n::stop_idx_t> find_stop_index(n::timetable const& tt,
                                             n::rt::frun const& run,
                                             unsigned const static_sequence) {
  auto const trip = run[0U].get_trip_idx(n::event_type::kDep);
  if (trip == n::trip_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const encoded = tt.trip_stop_seq_numbers_[trip];
  auto const sequence_range = n::loader::gtfs::stop_seq_number_range{
      {encoded.data(), encoded.size()},
      static_cast<n::stop_idx_t>(run.stop_range_.size())};
  auto const sequences =
      std::vector<n::stop_idx_t>{begin(sequence_range), end(sequence_range)};
  for (auto const [index, sequence] : utl::enumerate(sequences)) {
    if (static_cast<unsigned>(sequence) == static_sequence) {
      return static_cast<n::stop_idx_t>(index);
    }
  }
  return std::nullopt;
}

std::optional<prepared_trip> prepare_updates(
    n::timetable const& tt,
    n::rt_timetable const& rtt,
    selected_vehicle_prediction_trip const& trip,
    vehicle_prediction_overlay_policy const& policy,
    vehicle_prediction_overlay_state const& state) {
  auto run = n::rt::frun::from_t(tt, &rtt, trip.transport_);
  if (!run.valid()) {
    return std::nullopt;
  }
  if (trip.trip_stop_range_.has_value()) {
    if (trip.trip_stop_range_->empty() ||
        trip.trip_stop_range_->to_ > run.stop_range_.to_) {
      return std::nullopt;
    }
    run.stop_range_ = *trip.trip_stop_range_;
  }
  auto const stop_count = static_cast<n::stop_idx_t>(run.stop_range_.size());
  auto updates = std::vector<event_update>{};
  updates.reserve(trip.predictions_.size());
  for (auto const& prediction : trip.predictions_) {
    auto const stop_idx =
        find_stop_index(tt, run, prediction.static_stop_sequence_);
    if (!stop_idx.has_value()) {
      return std::nullopt;
    }
    auto const event_type =
        prediction.event_type_ == vehicle_prediction_event_type::kArrival
            ? n::event_type::kArr
            : n::event_type::kDep;
    if ((*stop_idx == 0U && event_type == n::event_type::kArr) ||
        (*stop_idx + 1U == stop_count && event_type == n::event_type::kDep)) {
      continue;
    }
    if (to_seconds(run[*stop_idx].scheduled_time(event_type)) !=
        prediction.scheduled_timestamp_seconds_) {
      return std::nullopt;
    }
    auto const delay_minutes = round_delay_minutes(
        prediction.delay_seconds_, previous_delay(state, trip, prediction),
        policy.minute_rounding_deadband_seconds_);
    auto delay_seconds = std::int64_t{};
    auto selected_seconds = std::int64_t{};
    if (__builtin_mul_overflow(delay_minutes, std::int64_t{60},
                               &delay_seconds) ||
        __builtin_add_overflow(prediction.scheduled_timestamp_seconds_,
                               delay_seconds, &selected_seconds)) {
      return std::nullopt;
    }
    if (event_type == n::event_type::kDep) {
      selected_seconds = std::max(
          selected_seconds, prediction.scheduled_timestamp_seconds_ -
                                policy.early_departure_tolerance_seconds_);
    }
    auto const selected_duration =
        event_type == n::event_type::kDep
            ? std::chrono::ceil<n::unixtime_t::duration>(
                  std::chrono::seconds{selected_seconds})
            : std::chrono::duration_cast<n::unixtime_t::duration>(
                  std::chrono::seconds{selected_seconds});
    auto const selected_time = n::unixtime_t{selected_duration};
    updates.push_back(
        {.stop_idx_ = *stop_idx,
         .static_stop_sequence_ = prediction.static_stop_sequence_,
         .event_type_ = event_type,
         .time_ = selected_time,
         .delay_ = selected_time - run[*stop_idx].scheduled_time(event_type)});
  }
  if (updates.empty()) {
    return std::nullopt;
  }

  auto effective_time = [&](n::stop_idx_t const stop,
                            n::event_type const event) {
    auto const update = std::ranges::find_if(updates, [&](auto const& x) {
      return x.stop_idx_ == stop && x.event_type_ == event;
    });
    return update == end(updates) ? run[stop].time(event) : update->time_;
  };
  for (auto const [index, stop] : utl::enumerate(run)) {
    auto const i = static_cast<n::stop_idx_t>(index);
    auto const arrival = i == 0U ? effective_time(i, n::event_type::kDep)
                                 : effective_time(i, n::event_type::kArr);
    auto const departure = i + 1U == stop_count
                               ? effective_time(i, n::event_type::kArr)
                               : effective_time(i, n::event_type::kDep);
    if (arrival > departure ||
        (i + 1U != stop_count &&
         departure > effective_time(i + 1U, n::event_type::kArr))) {
      return std::nullopt;
    }
  }
  return prepared_trip{.trip_ = &trip,
                       .range_ = run.stop_range_,
                       .updates_ = std::move(updates)};
}

n::stop_idx_t absolute_stop(prepared_trip const& trip,
                            event_update const& update) {
  return static_cast<n::stop_idx_t>(trip.range_.from_ + update.stop_idx_);
}

n::unixtime_t effective_time(n::rt::frun const& run,
                             std::span<prepared_trip const> const prepared,
                             n::stop_idx_t const stop_idx,
                             n::event_type const event_type) {
  for (auto const& trip : prepared) {
    auto const update = std::ranges::find_if(trip.updates_, [&](auto const& x) {
      return absolute_stop(trip, x) == stop_idx && x.event_type_ == event_type;
    });
    if (update != end(trip.updates_)) {
      return update->time_;
    }
  }
  return run[stop_idx].time(event_type);
}

bool valid_transport_chronology(n::timetable const& tt,
                                n::rt_timetable const& rtt,
                                n::transport const transport,
                                std::span<prepared_trip const> const prepared) {
  auto const run = n::rt::frun::from_t(tt, &rtt, transport);
  auto const stop_count = static_cast<std::size_t>(run.stop_range_.size());
  for (auto const [idx, _] : utl::enumerate(run)) {
    auto const stop_idx = static_cast<n::stop_idx_t>(idx);
    auto const arrival =
        idx == 0U
            ? effective_time(run, prepared, stop_idx, n::event_type::kDep)
            : effective_time(run, prepared, stop_idx, n::event_type::kArr);
    auto const departure =
        idx + 1U == stop_count
            ? effective_time(run, prepared, stop_idx, n::event_type::kArr)
            : effective_time(run, prepared, stop_idx, n::event_type::kDep);
    if (arrival > departure ||
        (idx + 1U != stop_count &&
         departure > effective_time(run, prepared,
                                    static_cast<n::stop_idx_t>(stop_idx + 1U),
                                    n::event_type::kArr))) {
      return false;
    }
  }
  return true;
}

void apply_prepared_trip(n::timetable const& tt,
                         n::rt_timetable& rtt,
                         prepared_trip const& prepared,
                         vehicle_prediction_overlay_policy const& policy,
                         vehicle_prediction_overlay_state& state) {
  auto const& trip = *prepared.trip_;
  auto rt_transport = rtt.resolve_rt(trip.transport_);
  if (rt_transport == n::rt_transport_idx_t::invalid()) {
    rt_transport = rtt.add_rt_transport(trip.source_, tt, trip.transport_);
  }
  auto const run = n::rt::run{.t_ = trip.transport_, .rt_ = rt_transport};
  for (auto const& update : prepared.updates_) {
    auto const stop_idx = absolute_stop(prepared, update);
    rtt.update_time(rt_transport, stop_idx, update.event_type_, update.time_);
    rtt.dispatch_delay(run, stop_idx, update.event_type_, update.delay_);
  }
  for (auto const& prediction : trip.predictions_) {
    auto const event_type =
        prediction.event_type_ == vehicle_prediction_event_type::kArrival
            ? n::event_type::kArr
            : n::event_type::kDep;
    auto const update = std::ranges::find_if(
        prepared.updates_, [&](event_update const& candidate) {
          return candidate.static_stop_sequence_ ==
                     prediction.static_stop_sequence_ &&
                 candidate.event_type_ == event_type;
        });
    remember_delay(
        state, trip, prediction,
        update == end(prepared.updates_)
            ? round_delay_minutes(prediction.delay_seconds_,
                                  previous_delay(state, trip, prediction),
                                  policy.minute_rounding_deadband_seconds_)
            : update->delay_.count(),
        update == end(prepared.updates_)
            ? std::nullopt
            : std::optional{to_seconds(update->time_)});
  }
}

}  // namespace

vehicle_prediction_overlay_result apply_vehicle_prediction_overlay(
    n::timetable const& tt,
    n::rt_timetable& rtt,
    std::span<selected_vehicle_prediction_trip const> const trips,
    vehicle_prediction_overlay_policy const& policy,
    vehicle_prediction_overlay_state& state) {
  auto result = vehicle_prediction_overlay_result{};
  std::erase_if(state.rendered_events_, [&](auto const& event) {
    return std::ranges::none_of(trips, [&](auto const& trip) {
      return trip.transport_ == event.transport_ &&
             trip.trip_stop_range_ == event.trip_stop_range_;
    });
  });
  auto groups =
      std::map<n::transport,
               std::vector<selected_vehicle_prediction_trip const*>>{};
  for (auto const& trip : trips) {
    groups[trip.transport_].push_back(&trip);
  }
  for (auto const& [transport, group] : groups) {
    auto prepared = std::vector<prepared_trip>{};
    prepared.reserve(group.size());
    for (auto const* trip : group) {
      auto updates = prepare_updates(tt, rtt, *trip, policy, state);
      if (updates.has_value()) {
        prepared.emplace_back(std::move(*updates));
      }
    }
    if (prepared.size() != group.size() ||
        !valid_transport_chronology(tt, rtt, transport, prepared)) {
      result.rejected_trips_ += group.size();
      continue;
    }
    std::ranges::sort(prepared, {}, [](prepared_trip const& trip) {
      return trip.range_.from_;
    });
    for (auto const& trip : prepared) {
      apply_prepared_trip(tt, rtt, trip, policy, state);
    }
    result.applied_trips_ += prepared.size();
    result.applied_transports_.push_back(transport);
  }
  return result;
}

}  // namespace motis
