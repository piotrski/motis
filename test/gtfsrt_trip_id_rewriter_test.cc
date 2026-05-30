#include "gtest/gtest.h"

#include <filesystem>

#include "date/date.h"

#include "nigiri/rt/create_rt_timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/gtfsrt_trip_id_rewriter.h"
#include "motis/import.h"

using namespace std::string_view_literals;
using namespace date;

constexpr auto const kGTFS = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
TEST,Test Agency,https://example.com,Europe/Warsaw

# stops.txt
stop_id,stop_name,stop_lat,stop_lon
A,Stop A,52.1,21.0
B,Stop B,52.2,21.1

# routes.txt
route_id,agency_id,route_short_name,route_long_name,route_type
219,TEST,219,Test 219,3

# trips.txt
route_id,service_id,trip_id,trip_headsign,direction_id
219,S1,static-trip-1,To B,0

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence
static-trip-1,21:06:00,21:06:00,A,0
static-trip-1,21:10:00,21:10:00,B,1

# calendar_dates.txt
service_id,date,exception_type
S1,20260530,1
)"sv;

TEST(motis, gtfsrt_trip_id_rewriter) {
  auto ec = std::error_code{};
  std::filesystem::remove_all("test/data", ec);

  auto const c = motis::config{
      .server_ = {{.web_folder_ = "ui/build", .n_threads_ = 1U}},
      .osm_ = {"test/resources/test_case.osm.pbf"},
      .timetable_ = motis::config::timetable{
          .first_day_ = "2026-05-30",
          .num_days_ = 2,
          .datasets_ = {{"test", {.path_ = std::string{kGTFS}}}}}};
  motis::import(c, "test/data");
  auto d = motis::data{"test/data", c};
  auto const rtt =
      nigiri::rt::create_rt_timetable(*d.tt_, date::sys_days{2026_y / May / 30});

  auto lookup =
      motis::build_gtfsrt_trip_id_lookup(*d.tt_, *d.tags_, rtt, "test");

  auto feed = transit_realtime::FeedMessage{};
  auto* entity = feed.add_entity();
  auto* trip = entity->mutable_trip_update()->mutable_trip();
  trip->set_trip_id("2026-05-30:219:SbS:3:2106");
  trip->set_route_id("219");
  trip->set_start_date("20260530");
  trip->set_start_time("21:06:00");

  ASSERT_TRUE(motis::rewrite_gtfsrt_trip_ids(feed, lookup));
  EXPECT_EQ("static-trip-1", trip->trip_id());
}
