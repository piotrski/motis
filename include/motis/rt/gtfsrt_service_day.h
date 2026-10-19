#pragma once

#include <cstddef>
#include <cstdint>

#include "nigiri/types.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

namespace nigiri {
struct timetable;
}  // namespace nigiri

namespace motis {

struct gtfsrt_service_day_stats {
  std::size_t inferred_{};
  std::size_t dropped_trip_updates_{};
  std::size_t cleared_vehicle_trips_{};
};

// GTFS-RT TripDescriptors without start_date are otherwise resolved against
// the UTC calendar date as GTFS service date. Night trips (times past 24:00)
// then land on the wrong service day: after UTC midnight a trip that runs
// every night resolves to the next night's run, and a trip id that only runs
// on the following service day resolves to a run 24 hours away.
//
// For every start_date-less descriptor whose trip_id is known, this picks the
// service day whose scheduled run is closest to `now` and writes it as
// start_date. Runs of one trip id repeat every 24 hours, so a run more than
// `max_distance` away cannot be the one a vehicle is serving or a feed
// announces ahead: the descriptor names a run that is not operating, its
// TripUpdate is dropped and its VehiclePosition keeps the vehicle but loses
// the trip.
gtfsrt_service_day_stats infer_gtfsrt_start_dates(
    nigiri::timetable const&,
    nigiri::source_idx_t,
    transit_realtime::FeedMessage&,
    std::int64_t now_seconds,
    std::int64_t max_distance_seconds = 20 * 60 * 60);

}  // namespace motis
