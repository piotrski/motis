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
  std::size_t not_operating_{};
};

// GTFS-RT TripDescriptors without start_date are otherwise resolved against
// the UTC calendar date as GTFS service date. Night trips (times past 24:00)
// then land on the wrong service day: after UTC midnight a trip that runs
// every night resolves to the next night's run, and a trip id that only runs
// on the following service day resolves to a run 24 hours away.
//
// For every start_date-less descriptor whose trip_id is known, this picks the
// service day whose scheduled run is closest to `now` and writes it as
// start_date. Runs of one trip id repeat every 24 hours, so when the closest
// run is more than `max_distance` away the descriptor names a run that is not
// operating (ZTP Kraków sends the next service day's trip id for post-midnight
// runs at schedule changes). Such descriptors are left as sent and counted:
// dropping them would also hide the vehicles, and mapping them to the run
// actually operating needs a trip-equivalence rule.
gtfsrt_service_day_stats infer_gtfsrt_start_dates(
    nigiri::timetable const&,
    nigiri::source_idx_t,
    transit_realtime::FeedMessage&,
    std::int64_t now_seconds,
    std::int64_t max_distance_seconds = 20 * 60 * 60);

}  // namespace motis
