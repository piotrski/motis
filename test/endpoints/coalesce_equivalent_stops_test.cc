#include "gtest/gtest.h"

#include <algorithm>
#include <filesystem>

#include "fmt/format.h"

#include "utl/helpers/algorithm.h"
#include "utl/init_from.h"
#include "utl/to_vec.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/endpoints/adr/geocode.h"
#include "motis/endpoints/map/stops.h"
#include "motis/endpoints/stop_times.h"
#include "motis/endpoints/trip.h"
#include "motis/import.h"

using namespace motis;

namespace {

constexpr auto const kFeedA = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
A,Agency A,https://a.example,Europe/Warsaw

# stops.txt
stop_id,stop_name,stop_lat,stop_lon,location_type,parent_station
RM,Rondo Mogilskie,50.067000,19.945000,0,
TS,Teatr Slowackiego,50.064500,19.941000,0,

# routes.txt
route_id,agency_id,route_short_name,route_long_name,route_desc,route_type
BUS,A,A1,,,3

# trips.txt
route_id,service_id,trip_id,trip_headsign,block_id
BUS,S1,A_TRIP,Teatr Slowackiego,

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type
A_TRIP,10:00:00,10:00:00,RM,1,0,0
A_TRIP,10:10:00,10:10:00,TS,2,0,0

# calendar_dates.txt
service_id,date,exception_type
S1,20200101,1
)";

constexpr auto const kFeedM = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
M,Agency M,https://m.example,Europe/Warsaw

# stops.txt
stop_id,stop_name,stop_lat,stop_lon,location_type,parent_station
RM,Rondo Mogilskie,50.067030,19.945030,0,
TS,Teatr Slowackiego,50.064530,19.941030,0,

# routes.txt
route_id,agency_id,route_short_name,route_long_name,route_desc,route_type
BUS,M,M1,,,3

# trips.txt
route_id,service_id,trip_id,trip_headsign,block_id
BUS,S1,M_TRIP,Teatr Slowackiego,

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type
M_TRIP,10:00:00,10:00:00,RM,1,0,0
M_TRIP,10:10:00,10:10:00,TS,2,0,0

# calendar_dates.txt
service_id,date,exception_type
S1,20200101,1
)";

constexpr auto const kFeedT = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
T,Agency T,https://t.example,Europe/Warsaw

# stops.txt
stop_id,stop_name,stop_lat,stop_lon,location_type,parent_station
RM,Rondo Mogilskie,50.067020,19.945020,0,
TS,Teatr Slowackiego,50.064520,19.941020,0,

# routes.txt
route_id,agency_id,route_short_name,route_long_name,route_desc,route_type
TRAM,T,T1,,,0

# trips.txt
route_id,service_id,trip_id,trip_headsign,block_id
TRAM,S1,T_TRIP,Teatr Slowackiego,

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type
T_TRIP,10:00:00,10:00:00,RM,1,0,0
T_TRIP,10:10:00,10:10:00,TS,2,0,0

# calendar_dates.txt
service_id,date,exception_type
S1,20200101,1
)";

config make_config(bool const coalesce_equivalent_stops) {
  return {
      .timetable_ =
          config::timetable{
              .first_day_ = "2020-01-01",
              .num_days_ = 2,
              .merge_dupes_inter_src_ = true,
              .coalesce_equivalent_stops_ = coalesce_equivalent_stops,
              .datasets_ = {
                  {"a", {.path_ = kFeedA}},
                  {"m", {.path_ = kFeedM}},
                  {"t", {.path_ = kFeedT}},
              }},
      .geocoding_ = true};
}

data import_fixture(std::string const& path, bool const coalesce_equivalent_stops) {
  auto ec = std::error_code{};
  std::filesystem::remove_all(path, ec);

  auto const c = make_config(coalesce_equivalent_stops);
  import(c, path);
  return {path, c};
}

}  // namespace

TEST(motis, coalesce_equivalent_stops) {
  auto d_off = import_fixture("test/data/coalesce-off", false);
  auto d_on = import_fixture("test/data/coalesce-on", true);

  auto const stops_off = utl::init_from<ep::stops>(d_off).value();
  auto const stops_on = utl::init_from<ep::stops>(d_on).value();
  auto const stop_times_off = utl::init_from<ep::stop_times>(d_off).value();
  auto const stop_times_on = utl::init_from<ep::stop_times>(d_on).value();
  auto const geocode_on = utl::init_from<ep::geocode>(d_on).value();
  auto const trip_on = utl::init_from<ep::trip>(d_on).value();

  auto const map_query =
      "/api/v1/map/stops?min=50.0600,19.9390&max=50.0700,19.9470";
  auto const map_off = stops_off(map_query);
  auto const map_on = stops_on(map_query);
  EXPECT_EQ(6, map_off.size());
  EXPECT_EQ(2, map_on.size());

  auto const stop_ids_on =
      utl::to_vec(map_on, [](api::Place const& p) { return *p.stopId_; });
  EXPECT_NE(end(stop_ids_on), utl::find(stop_ids_on, "a_RM"));
  EXPECT_NE(end(stop_ids_on), utl::find(stop_ids_on, "a_TS"));

  auto const stop_times_off_res =
      stop_times_off("/api/v5/stoptimes?stopId=m_RM&time=2020-01-01T08:55:00.000Z"
                     "&n=5");
  auto const stop_times_on_res =
      stop_times_on("/api/v5/stoptimes?stopId=m_RM&time=2020-01-01T08:55:00.000Z"
                    "&n=5");
  auto const stop_times_canonical_res =
      stop_times_on("/api/v5/stoptimes?stopId=a_RM&time=2020-01-01T08:55:00.000Z"
                    "&n=5");

  EXPECT_EQ("m_RM", stop_times_off_res.place_.stopId_);
  EXPECT_EQ("a_RM", stop_times_on_res.place_.stopId_);
  EXPECT_EQ("a_RM", stop_times_canonical_res.place_.stopId_);
  ASSERT_EQ(1, stop_times_on_res.stopTimes_.size());
  EXPECT_EQ(stop_times_on_res.stopTimes_.size(),
            stop_times_canonical_res.stopTimes_.size());
  EXPECT_EQ("a_RM", stop_times_on_res.stopTimes_.front().place_.stopId_);
  EXPECT_EQ("a_RM", stop_times_on_res.stopTimes_.front().tripFrom_.stopId_);
  EXPECT_EQ("a_TS", stop_times_on_res.stopTimes_.front().tripTo_.stopId_);

  auto const geocode_res = geocode_on("/api/v1/geocode?text=Rondo%20Mogilskie");
  auto const rondo =
      utl::find_if(geocode_res, [](api::Match const& m) { return m.id_ == "a_RM"; });
  ASSERT_NE(end(geocode_res), rondo);
  EXPECT_EQ(1, std::count_if(begin(geocode_res), end(geocode_res), [](api::Match const& m) {
              return m.id_ == "a_RM";
            }));
  ASSERT_TRUE(rondo->modes_.has_value());
  EXPECT_NE(end(*rondo->modes_), utl::find(*rondo->modes_, api::ModeEnum::BUS));
  EXPECT_NE(end(*rondo->modes_),
            utl::find(*rondo->modes_, api::ModeEnum::TRAM));

  auto trip_id = stop_times_on_res.stopTimes_.front().tripId_;
  for (auto pos = trip_id.find(':'); pos != std::string::npos;
       pos = trip_id.find(':', pos + 3U)) {
    trip_id.replace(pos, 1U, "%3A");
  }
  auto const trip_res = trip_on(fmt::format("/api/v5/trip?tripId={}", trip_id));
  ASSERT_EQ(1, trip_res.legs_.size());
  EXPECT_EQ("a_RM", trip_res.legs_.front().from_.stopId_);
  EXPECT_EQ("a_TS", trip_res.legs_.front().to_.stopId_);
}
