#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "date/date.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

#include "utl/init_from.h"

#include "nigiri/rt/create_rt_timetable.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/gtfsrt_update.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/endpoints/stop_times.h"
#include "motis/endpoints/trip.h"
#include "motis/import.h"
#include "motis/rt/passed_stops.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;
using namespace motis;
using namespace date;
using namespace std::chrono_literals;

namespace {

// T1 is a day trip, N1 a night trip crossing midnight UTC (the day rollover of
// the realtime timetable).
constexpr auto kGtfs = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
test,Test,https://example.test,UTC

# stops.txt
stop_id,stop_name,stop_lat,stop_lon
s1,S1,50.00,19.90
s2,S2,50.01,19.91
s3,S3,50.02,19.92
s4,S4,50.03,19.93

# routes.txt
route_id,agency_id,route_short_name,route_type
R1,test,1,3

# trips.txt
route_id,service_id,trip_id
R1,S1,T1
R1,S1,N1

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence
T1,10:00:00,10:00:00,s1,1
T1,10:05:00,10:05:00,s2,2
T1,10:10:00,10:10:00,s3,3
T1,10:15:00,10:15:00,s4,4
N1,23:50:00,23:50:00,s1,1
N1,23:55:00,23:55:00,s2,2
N1,24:00:00,24:00:00,s3,3
N1,24:10:00,24:10:00,s4,4

# calendar_dates.txt
service_id,date,exception_type
S1,20190501,1
)";

constexpr auto kDay = date::sys_days{2019_y / May / 1};
constexpr auto kT1 = "20190501_10:00_test_T1";

std::int64_t at(date::sys_seconds const t) {
  return t.time_since_epoch().count();
}

struct stop_update {
  std::optional<unsigned> seq_{};
  std::optional<std::string> stop_id_{};
  std::optional<std::int32_t> delay_{};  // arrival and departure
  std::optional<date::sys_seconds> time_{};  // arrival only, like ZTP trams
};

transit_realtime::FeedMessage feed(std::string const& trip_id,
                                   std::string const& start_time,
                                   std::vector<stop_update> const& updates,
                                   date::sys_seconds const timestamp) {
  auto msg = transit_realtime::FeedMessage{};
  msg.mutable_header()->set_gtfs_realtime_version("2.0");
  msg.mutable_header()->set_incrementality(
      transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
  msg.mutable_header()->set_timestamp(
      static_cast<std::uint64_t>(at(timestamp)));
  if (updates.empty()) {
    return msg;
  }
  auto* const entity = msg.add_entity();
  entity->set_id(trip_id);
  auto* const tu = entity->mutable_trip_update();
  tu->mutable_trip()->set_trip_id(trip_id);
  tu->mutable_trip()->set_start_date("20190501");
  tu->mutable_trip()->set_start_time(start_time);
  for (auto const& u : updates) {
    auto* const stu = tu->add_stop_time_update();
    if (u.seq_.has_value()) {
      stu->set_stop_sequence(*u.seq_);
    }
    if (u.stop_id_.has_value()) {
      stu->set_stop_id(*u.stop_id_);
    }
    if (u.delay_.has_value()) {
      stu->mutable_arrival()->set_delay(*u.delay_);
      stu->mutable_departure()->set_delay(*u.delay_);
    }
    if (u.time_.has_value()) {
      stu->mutable_arrival()->set_time(at(*u.time_));
    }
  }
  return msg;
}

struct cycle {
  date::sys_days today_;
  std::unique_ptr<n::rt_timetable> rtt_;
  passed_stop_store store_;
};

struct fixture {
  explicit fixture(std::string_view const path,
                   std::string const& gtfs = kGtfs) {
    auto ec = std::error_code{};
    std::filesystem::remove_all(path, ec);
    auto const c = config{
        .timetable_ = config::timetable{.first_day_ = "2019-05-01",
                                        .num_days_ = 3,
                                        .datasets_ = {{"test", {.path_ = gtfs}}}},
        .street_routing_ = false};
    import(c, path);
    d_.emplace(path, c);
  }

  // One realtime cycle: a fresh timetable from this cycle's feed, then the
  // passed-stop update against the previous cycle's published state.
  cycle run(date::sys_seconds const now,
            transit_realtime::FeedMessage const& msg,
            cycle const* previous,
            vehicle_prediction_diagnostics_store const* diagnostics = nullptr,
            std::span<vehicle_prediction_cycle_result const> candidates = {},
            std::span<selected_vehicle_prediction_trip const> overlays = {}) {
    auto const today = std::chrono::floor<date::days>(now);
    auto rtt = std::make_unique<n::rt_timetable>(
        n::rt::create_rt_timetable(*d_->tt_, today));
    n::rt::gtfsrt_update_msg(*d_->tt_, *rtt, n::source_idx_t{0}, "test", msg);
    auto const positions = trip_update_positions(n::source_idx_t{0}, msg);
    auto store = update_passed_stops(
        {.tt_ = *d_->tt_,
         .tags_ = *d_->tags_,
         .today_ = today,
         .now_seconds_ = at(now),
         .previous_ = previous == nullptr ? nullptr : &previous->store_,
         .previous_rtt_ = previous == nullptr ? nullptr : previous->rtt_.get(),
         .previous_diagnostics_ = diagnostics,
         .candidates_ = candidates,
         .overlays_ = overlays},
        *rtt, positions);
    return {today, std::move(rtt), std::move(store)};
  }

  n::transport transport(cycle const& c, std::string const& trip_id) const {
    auto const [r, _] = d_->tags_->get_trip(*d_->tt_, c.rtt_.get(), trip_id);
    return r.t_;
  }

  // Effective time of an event in a cycle's timetable, as UTC "HH:MM".
  std::string time(cycle const& c,
                   std::string const& trip_id,
                   n::stop_idx_t const stop,
                   n::event_type const ev) const {
    auto const t = transport(c, trip_id);
    auto const rt = c.rtt_->resolve_rt(t);
    auto const time = rt == n::rt_transport_idx_t::invalid()
                          ? d_->tt_->event_time(t, stop, ev)
                          : c.rtt_->unix_event_time(rt, stop, ev);
    return date::format("%H:%M", time);
  }

  std::optional<data> d_;
};

constexpr auto kArr = n::event_type::kArr;
constexpr auto kDep = n::event_type::kDep;

}  // namespace

TEST(passed_stops, feed_omitting_passed_stops_keeps_their_delay) {
  auto f = fixture{"test/passed-stops-sticky"};

  // 10:04: the bus is 2 min late and the feed still describes every stop.
  auto const c1 = f.run(kDay + 10h + 4min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 1U, .stop_id_ = "s1", .delay_ = 120}},
                             kDay + 10h + 4min),
                        nullptr);
  EXPECT_TRUE(c1.store_.events_.empty());
  EXPECT_EQ("10:07", f.time(c1, kT1, 1U, kArr));

  // 10:08: the bus passed s1 and s2; the feed now starts at s3, +3 min.
  auto const c2 = f.run(kDay + 10h + 8min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 180}},
                             kDay + 10h + 8min),
                        &c1);
  auto const t = f.transport(c2, kT1);
  ASSERT_EQ(3U, c2.store_.events_.size());  // s1 dep, s2 arr, s2 dep
  EXPECT_EQ("10:02", f.time(c2, kT1, 0U, kDep));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kArr));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kDep));
  EXPECT_EQ("10:13", f.time(c2, kT1, 2U, kArr));
  ASSERT_NE(nullptr, c2.store_.find(t, 1U, kArr));
  EXPECT_EQ(vehicle_prediction_source::kProvider,
            c2.store_.find(t, 1U, kArr)->source_);
  EXPECT_EQ(nullptr, c2.store_.find(t, 2U, kArr));

  // Later cycles keep the frozen times even though the delay grows.
  auto const c3 = f.run(kDay + 10h + 11min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 4U, .stop_id_ = "s4", .delay_ = 300}},
                             kDay + 10h + 11min),
                        &c2);
  EXPECT_EQ("10:02", f.time(c3, kT1, 0U, kDep));
  EXPECT_EQ("10:07", f.time(c3, kT1, 1U, kDep));
  EXPECT_EQ("10:11", f.time(c3, kT1, 2U, kArr));  // frozen at omission time
  EXPECT_EQ(c2.store_.find(t, 1U, kArr)->time_seconds_,
            c3.store_.find(t, 1U, kArr)->time_seconds_);

  // The trip leaves the feed: its stops are no longer passed-stop data.
  auto const c4 = f.run(kDay + 10h + 20min,
                        feed("T1", "10:00:00", {}, kDay + 10h + 20min), &c3);
  EXPECT_TRUE(c4.store_.events_.empty());
  EXPECT_TRUE(c4.store_.first_covered_.empty());
}

TEST(passed_stops, stops_passed_before_first_seen_stay_scheduled) {
  auto f = fixture{"test/passed-stops-first-seen"};
  // MOTIS first sees the trip when the feed already starts at s3.
  auto const c1 = f.run(kDay + 10h + 8min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 180}},
                             kDay + 10h + 8min),
                        nullptr);
  auto const c2 = f.run(kDay + 10h + 9min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 180}},
                             kDay + 10h + 9min),
                        &c1);
  EXPECT_TRUE(c2.store_.events_.empty());
  EXPECT_EQ("10:00", f.time(c2, kT1, 0U, kDep));
  EXPECT_EQ("10:05", f.time(c2, kT1, 1U, kArr));
}

TEST(passed_stops, caught_up_vehicle_keeps_chronology) {
  auto f = fixture{"test/passed-stops-caught-up"};
  // Predicted 5 min late at s2 ...
  auto const c1 = f.run(kDay + 10h + 4min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 2U, .stop_id_ = "s2", .delay_ = 300}},
                             kDay + 10h + 4min),
                        nullptr);
  EXPECT_EQ("10:10", f.time(c1, kT1, 1U, kArr));
  // ... but at 10:12 it is already reported 3 min early for s3. The frozen
  // s2 events must not run later than s3.
  auto const c2 = f.run(kDay + 10h + 12min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = -180}},
                             kDay + 10h + 12min),
                        &c1);
  EXPECT_EQ("10:07", f.time(c2, kT1, 2U, kArr));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kArr));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kDep));
  EXPECT_EQ("10:00", f.time(c2, kT1, 0U, kDep));
  ASSERT_NE(nullptr, c2.store_.find(f.transport(c2, kT1), 1U, kArr));
}

TEST(passed_stops, stop_id_only_feed_with_absolute_times) {
  auto f = fixture{"test/passed-stops-stop-id"};
  // ZTP tram style: no stop_sequence, no delay, arrival time only.
  auto const c1 = f.run(
      kDay + 10h + 4min,
      feed("T1", "10:00:00",
           {{.stop_id_ = "s2", .time_ = kDay + 10h + 7min},
            {.stop_id_ = "s3", .time_ = kDay + 10h + 12min}},
           kDay + 10h + 4min),
      nullptr);
  auto const c2 = f.run(
      kDay + 10h + 9min,
      feed("T1", "10:00:00", {{.stop_id_ = "s3", .time_ = kDay + 10h + 13min}},
           kDay + 10h + 9min),
      &c1);
  auto const t = f.transport(c2, kT1);
  EXPECT_EQ(nullptr, c2.store_.find(t, 0U, kDep));  // not covered in c1
  ASSERT_NE(nullptr, c2.store_.find(t, 1U, kArr));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kArr));
  EXPECT_EQ("10:13", f.time(c2, kT1, 2U, kArr));
}

TEST(passed_stops, gps_selected_stop_freezes_observed_passage) {
  auto f = fixture{"test/passed-stops-gps"};
  auto const c1 = f.run(kDay + 10h + 4min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 2U, .stop_id_ = "s2", .delay_ = 120}},
                             kDay + 10h + 4min),
                        nullptr);
  auto const t = f.transport(c1, kT1);
  auto const now = at(kDay + 10h + 9min);
  auto const scheduled_s2 = at(kDay + 10h + 5min);
  auto const diagnostics = [&](vehicle_prediction_source const effective) {
    auto entry = vehicle_prediction_diagnostic_entry{
        .transport_ = t,
        .static_stop_sequence_ = 2U,
        .event_type_ = vehicle_prediction_event_type::kArrival,
        .trip_id_ = kT1,
        .observed_at_seconds_ = now,
        .scheduled_timestamp_seconds_ = scheduled_s2,
        .gps_ = prediction_candidate_diagnostic{
            .source_ = vehicle_prediction_source::kGps,
            .predicted_timestamp_seconds_ = scheduled_s2 + 60},
        .effective_ = {.source_ = effective,
                       .predicted_timestamp_seconds_ = scheduled_s2 + 135}};
    return vehicle_prediction_diagnostics_store::build(true, {entry}, now);
  };
  auto candidate = vehicle_prediction_cycle_result{.feed_ = "test",
                                                   .trip_id_ = kT1};
  candidate.trip_stop_range_ = n::interval<n::stop_idx_t>{0U, 4U};
  candidate.batch_.transport_ = t;
  candidate.batch_.observed_passages_ = {
      {.static_stop_sequence_ = 2U,
       .observed_timestamp_seconds_ = at(kDay + 10h + 6min + 30s),
       .uncertainty_seconds_ = 20}};
  auto const candidates = std::vector{candidate};
  auto const next = feed("T1", "10:00:00",
                         {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 120}},
                         kDay + 10h + 9min);

  // Effective GPS: the observed passage wins over the last prediction.
  auto const gps_store = diagnostics(vehicle_prediction_source::kGps);
  auto const gps = f.run(kDay + 10h + 9min, next, &c1, gps_store.get(),
                         candidates);
  ASSERT_NE(nullptr, gps.store_.find(t, 1U, kArr));
  EXPECT_EQ(vehicle_prediction_source::kGps,
            gps.store_.find(t, 1U, kArr)->source_);
  EXPECT_EQ(at(kDay + 10h + 6min + 30s),
            gps.store_.find(t, 1U, kArr)->time_seconds_);
  EXPECT_EQ("10:06", f.time(gps, kT1, 1U, kArr));

  // Shadow: GPS was only recorded, so the shown provider prediction is
  // frozen, to the second.
  auto const shadow_store = diagnostics(vehicle_prediction_source::kProvider);
  auto const shadow = f.run(kDay + 10h + 9min, next, &c1, shadow_store.get(),
                            candidates);
  ASSERT_NE(nullptr, shadow.store_.find(t, 1U, kArr));
  EXPECT_EQ(vehicle_prediction_source::kProvider,
            shadow.store_.find(t, 1U, kArr)->source_);
  EXPECT_EQ(at(kDay + 10h + 7min + 15s),
            shadow.store_.find(t, 1U, kArr)->time_seconds_);
  EXPECT_EQ("10:07", f.time(shadow, kT1, 1U, kArr));
}

TEST(passed_stops, freeze_keeps_shown_source_and_prefers_gps_passage) {
  auto f = fixture{"test/passed-stops-shown-source"};
  // On time at s2: nothing but the schedule was shown for it.
  auto const c1 = f.run(kDay + 10h + 4min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 2U, .stop_id_ = "s2", .delay_ = 0}},
                             kDay + 10h + 4min),
                        nullptr);
  auto const t = f.transport(c1, kT1);
  auto const next = feed("T1", "10:00:00",
                         {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 60}},
                         kDay + 10h + 9min);
  auto const plain = f.run(kDay + 10h + 9min, next, &c1);
  ASSERT_NE(nullptr, plain.store_.find(t, 1U, kArr));
  EXPECT_EQ(vehicle_prediction_source::kSchedule,
            plain.store_.find(t, 1U, kArr)->source_);

  // GPS is selected for the vehicle this cycle (it has an overlay) and its
  // history observed the passage: a one-cycle schedule fallback at the stop
  // does not get frozen.
  auto candidate = vehicle_prediction_cycle_result{.feed_ = "test",
                                                   .trip_id_ = kT1};
  candidate.trip_stop_range_ = n::interval<n::stop_idx_t>{0U, 4U};
  candidate.batch_.transport_ = t;
  candidate.batch_.observed_passages_ = {
      {.static_stop_sequence_ = 2U,
       .observed_timestamp_seconds_ = at(kDay + 10h + 6min + 40s),
       .uncertainty_seconds_ = 15}};
  auto const candidates = std::vector{candidate};
  auto const overlays = std::vector{selected_vehicle_prediction_trip{
      .transport_ = t,
      .source_ = n::source_idx_t{0},
      .trip_stop_range_ = n::interval<n::stop_idx_t>{0U, 4U},
      .predictions_ = {{.static_stop_sequence_ = 3U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = at(kDay + 10h + 10min),
                        .predicted_timestamp_seconds_ = at(kDay + 10h + 11min),
                        .delay_seconds_ = 60}}}};
  auto const gps =
      f.run(kDay + 10h + 9min, next, &c1, nullptr, candidates, overlays);
  ASSERT_NE(nullptr, gps.store_.find(t, 1U, kArr));
  EXPECT_EQ(vehicle_prediction_source::kGps,
            gps.store_.find(t, 1U, kArr)->source_);
  EXPECT_EQ(at(kDay + 10h + 6min + 40s),
            gps.store_.find(t, 1U, kArr)->time_seconds_);
}

TEST(passed_stops, passed_events_stay_before_gps_overlay) {
  auto f = fixture{"test/passed-stops-overlay"};
  auto const c1 = f.run(kDay + 10h + 4min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 2U, .stop_id_ = "s2", .delay_ = 240}},
                             kDay + 10h + 4min),
                        nullptr);
  auto const t = f.transport(c1, kT1);
  // The provider still says s3 +4, the selected GPS overlay says s3 at 10:08.
  auto const overlays = std::vector{selected_vehicle_prediction_trip{
      .transport_ = t,
      .source_ = n::source_idx_t{0},
      .trip_stop_range_ = n::interval<n::stop_idx_t>{0U, 4U},
      .predictions_ = {{.static_stop_sequence_ = 3U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = at(kDay + 10h + 10min),
                        .predicted_timestamp_seconds_ =
                            at(kDay + 10h + 8min + 20s),
                        .delay_seconds_ = -100}}}};
  // An overlay of another trip on the same transport with the same stop
  // sequence (a later trip of the block) does not bound this trip.
  auto other_trip = overlays.front();
  other_trip.trip_stop_range_ = n::interval<n::stop_idx_t>{4U, 8U};
  other_trip.predictions_.front().predicted_timestamp_seconds_ =
      at(kDay + 10h + 1min);
  auto const with_other = std::vector{overlays.front(), other_trip};
  auto const c2 = f.run(kDay + 10h + 10min,
                        feed("T1", "10:00:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 240}},
                             kDay + 10h + 10min),
                        &c1, nullptr, {}, with_other);
  EXPECT_EQ("10:14", f.time(c2, kT1, 2U, kArr));  // overlay not applied here
  EXPECT_EQ("10:08", f.time(c2, kT1, 1U, kDep));
  EXPECT_EQ("10:08", f.time(c2, kT1, 1U, kArr));
}

TEST(passed_stops, night_trip_keeps_passed_stops_across_day_rollover) {
  auto f = fixture{"test/passed-stops-rollover"};
  constexpr auto kN1 = "20190501_23:50_test_N1";
  auto const c1 = f.run(kDay + 23h + 54min,
                        feed("N1", "23:50:00",
                             {{.seq_ = 2U, .stop_id_ = "s2", .delay_ = 120}},
                             kDay + 23h + 54min),
                        nullptr);
  // First cycle after midnight UTC builds a timetable for the next day.
  auto const c2 = f.run(kDay + 24h + 1min,
                        feed("N1", "23:50:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 120}},
                             kDay + 24h + 1min),
                        &c1);
  ASSERT_NE(c1.today_, c2.today_);
  auto const t = f.transport(c2, kN1);
  ASSERT_NE(nullptr, c2.store_.find(t, 1U, kArr));
  EXPECT_EQ("23:57", f.time(c2, kN1, 1U, kArr));
  auto const c3 = f.run(kDay + 24h + 3min,
                        feed("N1", "23:50:00",
                             {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 120}},
                             kDay + 24h + 3min),
                        &c2);
  EXPECT_EQ("23:57", f.time(c3, kN1, 1U, kArr));
}

TEST(passed_stops, trip_response_marks_passed_stops) {
  auto f = fixture{"test/passed-stops-trip"};
  auto c1 = f.run(kDay + 10h + 4min,
                  feed("T1", "10:00:00",
                       {{.seq_ = 1U, .stop_id_ = "s1", .delay_ = 120}},
                       kDay + 10h + 4min),
                  nullptr);
  auto c2 = f.run(kDay + 10h + 8min,
                  feed("T1", "10:00:00",
                       {{.seq_ = 3U, .stop_id_ = "s3", .delay_ = 180}},
                       kDay + 10h + 8min),
                  &c1);
  // A frozen time with seconds is shown to the second.
  auto const t = f.transport(c2, kT1);
  c2.store_.events_.at({t, 1U, kDep}).time_seconds_ =
      at(kDay + 10h + 7min + 20s);
  auto& d = *f.d_;
  d.init_rtt(kDay);
  d.rt_->rtt_ = std::move(c2.rtt_);
  d.rt_->passed_stops_ =
      std::make_unique<passed_stop_store>(std::move(c2.store_));
  auto const trip_ep = utl::init_from<ep::trip>(d).value();

  auto const res = trip_ep("/api/v6/trip?tripId=20190501_10%3A00_test_T1");
  ASSERT_EQ(1U, res.legs_.size());
  auto const& leg = res.legs_.front();
  using status = api::PredictionStatusEnum;
  ASSERT_TRUE(leg.from_.departurePrediction_.has_value());
  EXPECT_EQ(status::PASSED, leg.from_.departurePrediction_->status_);
  EXPECT_EQ(api::PredictionSourceEnum::PROVIDER,
            leg.from_.departurePrediction_->source_);
  EXPECT_EQ(120, leg.from_.departurePrediction_->delaySeconds_);
  ASSERT_EQ(2U, leg.intermediateStops_->size());
  auto const& s2 = leg.intermediateStops_->at(0);
  EXPECT_EQ(status::PASSED, s2.arrivalPrediction_->status_);
  EXPECT_EQ(status::PASSED, s2.departurePrediction_->status_);
  EXPECT_EQ("2019-05-01T10:07:00Z", s2.arrivalPrediction_->time_);
  EXPECT_EQ("2019-05-01T10:07:20Z", s2.departurePrediction_->time_);
  EXPECT_EQ(140, s2.departurePrediction_->delaySeconds_);
  auto const& s3 = leg.intermediateStops_->at(1);
  EXPECT_FALSE(s3.arrivalPrediction_->status_.has_value());
  EXPECT_EQ(180, s3.arrivalPrediction_->delaySeconds_);
  EXPECT_FALSE(leg.to_.arrivalPrediction_->status_.has_value());
  ASSERT_TRUE(res.startPrediction_.has_value());
  EXPECT_EQ(status::PASSED, res.startPrediction_->status_);
}

TEST(passed_stops, malformed_entity_does_not_stop_other_trips) {
  auto f = fixture{"test/passed-stops-bad-entity"};
  // A route-only descriptor without the start date and time nigiri needs to
  // resolve it: nigiri skips it when applying the feed, and so must the
  // passed-stop update, without affecting the healthy entity next to it.
  auto const with_bad_entity = [](transit_realtime::FeedMessage msg) {
    auto* const bad = msg.add_entity();
    bad->set_id("unsupported-route-only");
    bad->mutable_trip_update()->mutable_trip()->set_route_id("R1");
    bad->mutable_trip_update()->add_stop_time_update()->set_stop_sequence(1U);
    return msg;
  };
  auto const c1 = f.run(
      kDay + 10h + 4min,
      with_bad_entity(feed("T1", "10:00:00", {{.seq_ = 2U, .delay_ = 120}},
                           kDay + 10h + 4min)),
      nullptr);
  auto const c2 = f.run(
      kDay + 10h + 8min,
      with_bad_entity(feed("T1", "10:00:00", {{.seq_ = 3U, .delay_ = 180}},
                           kDay + 10h + 8min)),
      &c1);
  auto const t = f.transport(c2, kT1);
  ASSERT_NE(nullptr, c2.store_.find(t, 1U, kArr));
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kArr));
}

TEST(passed_stops, stoptimes_marks_passed_events_of_realtime_runs) {
  // The stop time iterator hands out realtime runs without the static
  // transport; the passed-stop lookup must use the resolved one.
  auto f = fixture{"test/passed-stops-stoptimes"};
  auto c1 = f.run(
      kDay + 10h + 4min,
      feed("T1", "10:00:00", {{.seq_ = 2U, .delay_ = 120}}, kDay + 10h + 4min),
      nullptr);
  auto const t = f.transport(c1, kT1);
  auto entries = std::vector<vehicle_prediction_diagnostic_entry>{
      {.transport_ = t,
       .static_stop_sequence_ = 2U,
       .event_type_ = vehicle_prediction_event_type::kDeparture,
       .trip_id_ = kT1,
       .observed_at_seconds_ = at(kDay + 10h + 4min),
       .scheduled_timestamp_seconds_ = at(kDay + 10h + 5min),
       .effective_ = {
           .source_ = vehicle_prediction_source::kProvider,
           .predicted_timestamp_seconds_ = at(kDay + 10h + 7min + 50s)}}};
  auto const prev = vehicle_prediction_diagnostics_store::build(
      true, entries, at(kDay + 10h + 4min));
  auto c2 = f.run(
      kDay + 10h + 9min,
      feed("T1", "10:00:00", {{.seq_ = 3U, .delay_ = 180}}, kDay + 10h + 9min),
      &c1, prev.get());
  auto& d = *f.d_;
  d.init_rtt(kDay);
  d.rt_->rtt_ = std::move(c2.rtt_);
  d.rt_->passed_stops_ =
      std::make_unique<passed_stop_store>(std::move(c2.store_));
  ASSERT_NE(nullptr, d.rt_->passed_stops_->find(t, 1U, kDep));
  EXPECT_EQ(at(kDay + 10h + 7min + 50s),
            d.rt_->passed_stops_->find(t, 1U, kDep)->time_seconds_);
  auto const ep = utl::init_from<ep::stop_times>(d).value();
  auto const before =
      ep("/api/v6/stoptimes?stopId=test_s2&time=2019-05-01T10:07:00Z&n=1");
  ASSERT_EQ(1U, before.stopTimes_.size());
  EXPECT_EQ(kT1, before.stopTimes_[0].tripId_);
  EXPECT_EQ(api::PredictionStatusEnum::PASSED,
            before.stopTimes_[0].selectedPrediction_->status_);
  EXPECT_EQ("2019-05-01T10:07:50Z",
            before.stopTimes_[0].selectedPrediction_->time_);
  auto const after =
      ep("/api/v6/stoptimes?stopId=test_s2&time=2019-05-01T10:07:30Z&n=1");
  ASSERT_EQ(1U, after.stopTimes_.size());
  EXPECT_EQ(kT1, after.stopTimes_[0].tripId_);
}

TEST(passed_stops, repeated_stop_id_keeps_passed_visits) {
  // T1 becomes s1 -> s2 -> s1 -> s4: a stop_id-only update for the second s1
  // must not be taken for the first visit and drop the frozen ones.
  auto gtfs = std::string{kGtfs};
  auto const old = std::string{"T1,10:10:00,10:10:00,s3,3"};
  gtfs.replace(gtfs.find(old), old.size(), "T1,10:10:00,10:10:00,s1,3");
  auto f = fixture{"test/passed-stops-loop", gtfs};
  auto c1 = f.run(
      kDay + 10h + 1min,
      feed("T1", "10:00:00", {{.stop_id_ = "s1", .time_ = kDay + 10h + 2min}},
           kDay + 10h + 1min),
      nullptr);
  auto c2 = f.run(
      kDay + 10h + 4min,
      feed("T1", "10:00:00", {{.stop_id_ = "s2", .time_ = kDay + 10h + 7min}},
           kDay + 10h + 4min),
      &c1);
  auto c3 = f.run(
      kDay + 10h + 9min,
      feed("T1", "10:00:00", {{.stop_id_ = "s1", .time_ = kDay + 10h + 12min}},
           kDay + 10h + 9min),
      &c2);
  auto const t = f.transport(c3, kT1);
  ASSERT_NE(nullptr, c2.store_.find(t, 0U, kDep));
  EXPECT_NE(nullptr, c3.store_.find(t, 0U, kDep));
  EXPECT_NE(nullptr, c3.store_.find(t, 1U, kArr));
  EXPECT_EQ("10:02", f.time(c3, kT1, 0U, kDep));
  EXPECT_EQ("10:07", f.time(c3, kT1, 1U, kArr));
}

TEST(passed_stops, clamped_passed_time_does_not_move_back_later) {
  // Clamped to 10:07 when the bus was reported 3 min early at s3; a later
  // cycle with a growing delay at s3 must not move the passed s2 back.
  auto f = fixture{"test/passed-stops-clamp-persists"};
  auto c1 = f.run(
      kDay + 10h + 4min,
      feed("T1", "10:00:00", {{.seq_ = 2U, .delay_ = 300}}, kDay + 10h + 4min),
      nullptr);
  auto c2 = f.run(kDay + 10h + 12min,
                  feed("T1", "10:00:00", {{.seq_ = 3U, .delay_ = -180}},
                       kDay + 10h + 12min),
                  &c1);
  auto c3 = f.run(
      kDay + 10h + 13min,
      feed("T1", "10:00:00", {{.seq_ = 3U, .delay_ = 300}}, kDay + 10h + 13min),
      &c2);
  EXPECT_EQ("10:07", f.time(c2, kT1, 1U, kArr));
  EXPECT_EQ(f.time(c2, kT1, 1U, kArr), f.time(c3, kT1, 1U, kArr));
  auto const t = f.transport(c3, kT1);
  ASSERT_NE(nullptr, c3.store_.find(t, 1U, kArr));
  EXPECT_EQ(at(kDay + 10h + 7min), c3.store_.find(t, 1U, kArr)->time_seconds_);
}
