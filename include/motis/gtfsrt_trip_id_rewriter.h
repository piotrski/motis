#pragma once

#include <string>
#include <string_view>
#include <unordered_map>

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

#include "nigiri/rt/rt_timetable.h"
#include "nigiri/timetable.h"

#include "motis/tag_lookup.h"

namespace motis {

using gtfsrt_trip_id_lookup_t = std::unordered_map<std::string, std::string>;

gtfsrt_trip_id_lookup_t build_gtfsrt_trip_id_lookup(
    nigiri::timetable const&,
    tag_lookup const&,
    nigiri::rt_timetable const&,
    std::string_view tag);

bool rewrite_gtfsrt_trip_ids(transit_realtime::FeedMessage&,
                             gtfsrt_trip_id_lookup_t const&);

}  // namespace motis
