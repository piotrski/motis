#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include "date/date.h"

#include "nigiri/types.h"

#include "motis/rt/vehicle_prediction_diagnostics.h"
#include "motis/rt/vehicle_prediction_overlay.h"
#include "motis/rt/vehicle_prediction_store.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

namespace nigiri {
struct rt_timetable;
struct timetable;
}  // namespace nigiri

namespace motis {

struct tag_lookup;

// Where a GTFS-RT TripUpdate starts: feeds such as ZTP Kraków only describe
// the stops a vehicle has not passed yet, so the first stop time update marks
// the vehicle's position on its trip.
struct trip_update_position {
  nigiri::source_idx_t src_{nigiri::source_idx_t::invalid()};
  transit_realtime::TripDescriptor trip_{};
  std::optional<std::uint32_t> first_stop_sequence_{};
  std::optional<std::string> first_stop_id_{};
  std::optional<std::int64_t> reported_at_seconds_{};
};

[[nodiscard]] std::vector<trip_update_position> trip_update_positions(
    nigiri::source_idx_t, transit_realtime::FeedMessage const&);

struct passed_stop_event {
  // Last predicted or observed time of the event, frozen when the feed first
  // stopped describing the stop. Never later than that moment.
  std::int64_t time_seconds_{};
  vehicle_prediction_source source_{vehicle_prediction_source::kProvider};
  std::int64_t frozen_at_seconds_{};

  friend bool operator==(passed_stop_event const&,
                         passed_stop_event const&) = default;
};

// Stop events a vehicle has passed on the trips it currently serves, keyed by
// the exact stop visit (transport + absolute stop index). Rebuilt every cycle
// from the previous cycle's store, so entries live exactly as long as their
// trip stays in the feed.
struct passed_stop_store {
  struct event_key {
    nigiri::transport transport_{};
    nigiri::stop_idx_t stop_idx_{};
    nigiri::event_type event_type_{};

    friend bool operator<(event_key const& a, event_key const& b) {
      return std::tie(a.transport_, a.stop_idx_, a.event_type_) <
             std::tie(b.transport_, b.stop_idx_, b.event_type_);
    }
  };
  struct trip_key {
    nigiri::transport transport_{};
    nigiri::stop_idx_t trip_from_{};

    friend bool operator<(trip_key const& a, trip_key const& b) {
      return std::tie(a.transport_, a.trip_from_) <
             std::tie(b.transport_, b.trip_from_);
    }
  };

  [[nodiscard]] passed_stop_event const* find(nigiri::transport,
                                              nigiri::stop_idx_t,
                                              nigiri::event_type) const;

  std::map<event_key, passed_stop_event> events_{};
  // First stop index the feed still described for each trip in the cycle.
  // Only stops covered in the previous cycle can be frozen, so stops passed
  // before MOTIS saw the trip never get a made-up time.
  std::map<trip_key, nigiri::stop_idx_t> first_covered_{};
};

struct passed_stop_update_context {
  nigiri::timetable const& tt_;
  tag_lookup const& tags_;
  date::sys_days today_;
  std::int64_t now_seconds_{};
  // Published state of the previous cycle.
  passed_stop_store const* previous_{nullptr};
  nigiri::rt_timetable const* previous_rtt_{nullptr};
  vehicle_prediction_diagnostics_store const* previous_diagnostics_{nullptr};
  // This cycle's GPS candidates (observed passages) and selected overlays.
  std::span<vehicle_prediction_cycle_result const> candidates_{};
  std::span<selected_vehicle_prediction_trip const> overlays_{};
};

// Freezes the events of stops the feed no longer describes and writes them
// into `rtt` (the provider baseline built from this cycle's feeds), clamped so
// that passed events never run later than the vehicle's next event. Returns
// the store to publish with this cycle.
[[nodiscard]] passed_stop_store update_passed_stops(
    passed_stop_update_context const&,
    nigiri::rt_timetable& rtt,
    std::span<trip_update_position const>);

}  // namespace motis
