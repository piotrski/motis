#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>

#include "date/date.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/import.h"
#include "motis/rt/gtfsrt_service_day.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;
using namespace motis;
using namespace date;
using namespace std::chrono_literals;

namespace {

// NIGHT runs at 24:15 on both service days (00:15 the morning after). NEW is
// the same run under a new schedule version that only starts on 2 May, the
// id ZTP Kraków already sends for the night of 1 to 2 May.
constexpr auto kGtfs = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
test,Test,https://example.test,UTC

# stops.txt
stop_id,stop_name,stop_lat,stop_lon
s1,S1,50.00,19.90
s2,S2,50.01,19.91

# routes.txt
route_id,agency_id,route_short_name,route_type
R1,test,901,3

# trips.txt
route_id,service_id,trip_id
R1,BOTH,NIGHT
R1,SECOND,NEW

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence
NIGHT,24:15:00,24:15:00,s1,1
NIGHT,24:40:00,24:40:00,s2,2
NEW,24:15:00,24:15:00,s1,1
NEW,24:40:00,24:40:00,s2,2

# calendar_dates.txt
service_id,date,exception_type
BOTH,20190501,1
BOTH,20190502,1
SECOND,20190502,1
)";

constexpr auto kMay1 = date::sys_days{2019_y / May / 1};

std::int64_t at(date::sys_seconds const t) {
  return t.time_since_epoch().count();
}

transit_realtime::FeedMessage feed(std::string const& trip_id,
                                   std::optional<std::string> start_date = {}) {
  auto msg = transit_realtime::FeedMessage{};
  msg.mutable_header()->set_gtfs_realtime_version("2.0");
  auto* const tu = msg.add_entity();
  tu->set_id("tu");
  tu->mutable_trip_update()->mutable_trip()->set_trip_id(trip_id);
  auto* const vp = msg.add_entity();
  vp->set_id("vp");
  vp->mutable_vehicle()->mutable_trip()->set_trip_id(trip_id);
  vp->mutable_vehicle()->mutable_vehicle()->set_id("bus-1");
  if (start_date.has_value()) {
    tu->mutable_trip_update()->mutable_trip()->set_start_date(*start_date);
    vp->mutable_vehicle()->mutable_trip()->set_start_date(*start_date);
  }
  auto* const stu = tu->mutable_trip_update()->add_stop_time_update();
  stu->set_stop_sequence(2U);
  stu->mutable_arrival()->set_delay(120);
  return msg;
}

struct fixture {
  fixture() {
    auto ec = std::error_code{};
    std::filesystem::remove_all("test/gtfsrt-service-day", ec);
    auto const c = config{
        .timetable_ = config::timetable{.first_day_ = "2019-04-30",
                                        .num_days_ = 5,
                                        .datasets_ = {{"test", {.path_ = kGtfs}}}},
        .street_routing_ = false};
    import(c, "test/gtfsrt-service-day");
    d_.emplace("test/gtfsrt-service-day", c);
  }

  // Scheduled first departure of the run nigiri resolves the descriptor to.
  std::optional<date::sys_seconds> resolved_departure(
      transit_realtime::TripDescriptor const& td,
      date::sys_seconds const now) const {
    auto const [r, _] = n::rt::gtfsrt_resolve_run(
        std::chrono::floor<date::days>(now), *d_->tt_, nullptr,
        n::source_idx_t{0}, td);
    if (!r.valid()) {
      return std::nullopt;
    }
    return std::chrono::sys_seconds{std::chrono::seconds{to_seconds(
        d_->tt_->event_time(r.t_, r.stop_range_.from_, n::event_type::kDep))}};
  }

  std::optional<data> d_;
};

}  // namespace

TEST(gtfsrt_service_day, night_trip_after_utc_midnight_keeps_its_service_day) {
  auto const f = fixture{};
  auto const now = date::sys_seconds{kMay1 + 24h + 20min};  // 2 May 00:20

  // Without a start_date, the UTC date picks service day 2 May: the run of
  // the following night.
  auto const raw = feed("NIGHT");
  EXPECT_EQ(kMay1 + 48h + 15min,
            f.resolved_departure(raw.entity(0).trip_update().trip(), now));

  auto msg = feed("NIGHT");
  auto const stats = infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0},
                                              msg, at(now));
  EXPECT_EQ(2U, stats.inferred_);
  ASSERT_EQ(2, msg.entity_size());
  EXPECT_EQ("20190501", msg.entity(0).trip_update().trip().start_date());
  EXPECT_EQ("20190501", msg.entity(1).vehicle().trip().start_date());
  EXPECT_EQ(kMay1 + 24h + 15min,
            f.resolved_departure(msg.entity(0).trip_update().trip(), now));
}

TEST(gtfsrt_service_day, night_trip_before_midnight_uses_that_evening) {
  auto const f = fixture{};
  auto const now = date::sys_seconds{kMay1 + 23h + 50min};
  auto msg = feed("NIGHT");
  static_cast<void>(
      infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0}, msg, at(now)));
  EXPECT_EQ("20190501", msg.entity(0).trip_update().trip().start_date());
}

TEST(gtfsrt_service_day, trip_id_of_a_later_service_day_is_left_as_sent) {
  auto const f = fixture{};
  // The feed names tonight's run with an id that only runs from service day
  // 2 May (00:15 on 3 May): without inference it resolves 24 hours ahead.
  auto const now = date::sys_seconds{kMay1 + 24h + 20min};
  auto const raw = feed("NEW");
  EXPECT_EQ(kMay1 + 48h + 15min,
            f.resolved_departure(raw.entity(0).trip_update().trip(), now));

  // No run of NEW operates around now: the descriptors are left as sent
  // (and counted) rather than dropped, so the vehicle stays visible.
  auto msg = feed("NEW");
  auto const stats = infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0},
                                              msg, at(now));
  EXPECT_EQ(0U, stats.inferred_);
  EXPECT_EQ(2U, stats.not_operating_);
  ASSERT_EQ(2, msg.entity_size());
  EXPECT_FALSE(msg.entity(0).trip_update().trip().has_start_date());
  EXPECT_TRUE(msg.entity(1).vehicle().has_trip());

  // The next evening the same id is the operating run and is kept.
  auto next = feed("NEW");
  auto const next_now = date::sys_seconds{kMay1 + 48h + 20min};
  static_cast<void>(infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0},
                                             next, at(next_now)));
  ASSERT_EQ(2, next.entity_size());
  EXPECT_EQ("20190502", next.entity(0).trip_update().trip().start_date());
}

TEST(gtfsrt_service_day, explicit_start_date_and_unknown_trips_are_untouched) {
  auto const f = fixture{};
  auto const now = date::sys_seconds{kMay1 + 24h + 20min};
  auto explicit_date = feed("NIGHT", "20190502");
  auto unknown = feed("UNKNOWN");
  auto const a = infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0},
                                          explicit_date, at(now));
  auto const b = infer_gtfsrt_start_dates(*f.d_->tt_, n::source_idx_t{0},
                                          unknown, at(now));
  EXPECT_EQ(0U, a.inferred_ + a.not_operating_ + b.inferred_ +
                    b.not_operating_);
  EXPECT_EQ("20190502", explicit_date.entity(0).trip_update().trip().start_date());
  EXPECT_FALSE(unknown.entity(0).trip_update().trip().has_start_date());
  EXPECT_EQ(2, unknown.entity_size());
}
