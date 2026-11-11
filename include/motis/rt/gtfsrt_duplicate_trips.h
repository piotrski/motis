#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "nigiri/types.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

namespace nigiri {
struct timetable;
}  // namespace nigiri

namespace motis {

// GPS older than this does not show a vehicle on a trip.
constexpr auto kDuplicateTripMaxGpsAgeSeconds = std::int64_t{120};

struct duplicate_trip_stats {
  // Runs with more than one SCHEDULED TripUpdate.
  std::size_t trips_{};
  // TripUpdates turned into DUPLICATED copies.
  std::size_t copies_{};
  // TripUpdates removed because their vehicle is not on the run.
  std::size_t dropped_{};
};

// Some feeds (ZTP Kraków, 2026-11-10 line 139) send two TripUpdates for one
// run when a second vehicle stands in. nigiri would apply both to the same
// run, last writer wins per stop, so the run mixes two vehicles' times.
//
// For every run with more than one SCHEDULED TripUpdate in one message, this
// decides which vehicle keeps the scheduled run, using the VehiclePositions
// of all `messages` (the TripUpdates and VehiclePositions feeds of one
// dataset):
//  - A vehicle is on the run if its VehiclePosition names the run and is at
//    most `max_gps_age_seconds` old at `now_seconds`.
//  - Among the vehicles on the run, the one closest to schedule keeps the
//    scheduled TripUpdate. Each other one becomes a DUPLICATED copy with
//    trip_properties.trip_id `<trip_id>~<vehicle>`, and its VehiclePosition
//    names the copy.
//  - TripUpdates of vehicles not on the run are removed (forecasts for a
//    vehicle's next run, or a vehicle that stopped reporting). If no vehicle
//    is on the run yet, the TripUpdate of a vehicle with fresh GPS elsewhere
//    that is closest to schedule stays, and the others are removed.
//
// A vehicle is named by its licence plate where the feed has one (ZTP fleet
// numbers repeat across its bus and tram feeds), else by its vehicle id.
// TripUpdates are joined to VehiclePositions by vehicle id. Runs whose
// TripUpdates name no vehicle are left as sent. Running it again on its own
// output changes nothing.
duplicate_trip_stats split_duplicate_trip_updates(
    nigiri::timetable const&,
    nigiri::source_idx_t,
    std::span<transit_realtime::FeedMessage* const> messages,
    std::int64_t now_seconds,
    std::int64_t max_gps_age_seconds = kDuplicateTripMaxGpsAgeSeconds);

}  // namespace motis
