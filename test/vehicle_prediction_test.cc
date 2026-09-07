#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "nigiri/rt/create_rt_timetable.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/rt_timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/import.h"
#include "motis/rt/trip_progress_projection.h"
#include "motis/rt/vehicle_eta_control.h"
#include "motis/rt/vehicle_position.h"
#include "motis/rt/vehicle_prediction.h"
#include "motis/rt/vehicle_prediction_continuation.h"
#include "motis/rt/vehicle_prediction_diagnostics.h"
#include "motis/rt/vehicle_prediction_overlay.h"
#include "motis/rt/vehicle_prediction_store.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace fs = std::filesystem;
namespace n = nigiri;

namespace motis {
namespace {

constexpr auto const kGtfs = R"(
# agency.txt
agency_id,agency_name,agency_url,agency_timezone
Test,Test,https://example.com,Europe/Berlin

# stops.txt
stop_id,stop_name,stop_lat,stop_lon
A,A,50.0000,8.0000
B,B,50.0000,8.0100
C,C,50.0000,8.0200
D,D,50.0000,8.0300
E,E,50.0000,8.0400
F,F,50.0000,8.0500

# routes.txt
route_id,agency_id,route_short_name,route_type
route,Test,1,3
route2,Test,2,3
route3,Test,3,3

# trips.txt
route_id,service_id,trip_id,shape_id,block_id,trip_headsign
route,S1,straight,straight-shape,prediction-block,Incoming
route2,S1,continuation,continuation-shape,prediction-block,Outgoing
route3,S1,third,third-shape,prediction-block,Final

# stop_times.txt
trip_id,arrival_time,departure_time,stop_id,stop_sequence
straight,01:00:00,01:00:00,A,10
straight,01:05:00,01:05:00,B,20
straight,01:10:00,01:10:00,C,30
continuation,01:10:00,01:10:00,C,10
continuation,01:15:00,01:15:00,D,20
continuation,01:20:00,01:20:00,E,30
third,01:20:00,01:20:00,E,10
third,01:25:00,01:25:00,F,20

# calendar_dates.txt
service_id,date,exception_type
S1,20260521,1

# shapes.txt
shape_id,shape_pt_lat,shape_pt_lon,shape_pt_sequence
straight-shape,50.0000,8.0000,1
straight-shape,50.0000,8.0050,2
straight-shape,50.0000,8.0100,3
straight-shape,50.0000,8.0150,4
straight-shape,50.0000,8.0200,5
continuation-shape,50.0000,8.0200,1
continuation-shape,50.0000,8.0250,2
continuation-shape,50.0000,8.0300,3
continuation-shape,50.0000,8.0350,4
continuation-shape,50.0000,8.0400,5
third-shape,50.0000,8.0400,1
third-shape,50.0000,8.0450,2
third-shape,50.0000,8.0500,3
)";

struct prediction_fixture {
  prediction_fixture()
      : path_{fs::temp_directory_path() / "motis-vehicle-prediction-test"},
        config_{.timetable_ = config::timetable{
                    .first_day_ = "2026-05-21",
                    .num_days_ = 2,
                    .with_shapes_ = true,
                    .datasets_ = {{"prediction", {.path_ = kGtfs}}}}} {
    auto ec = std::error_code{};
    fs::remove_all(path_, ec);
    import(config_, path_);
    data_.emplace(path_, config_);
  }

  ~prediction_fixture() {
    data_.reset();
    auto ec = std::error_code{};
    fs::remove_all(path_, ec);
  }

  n::rt::frun run(std::string_view const trip_id = "straight",
                  std::string_view const start_time = "01:00") const {
    auto const [run, trip] =
        data_->tags_->get_trip(*data_->tt_, nullptr,
                               "20260521_" + std::string{start_time} +
                                   "_prediction_" + std::string{trip_id});
    EXPECT_TRUE(run.valid());
    EXPECT_NE(trip, n::trip_idx_t::invalid());
    return n::rt::frun{*data_->tt_, nullptr, run};
  }

  std::int64_t next_scheduled() const {
    return std::chrono::duration_cast<std::chrono::seconds>(
               run()[1].scheduled_time(n::event_type::kArr).time_since_epoch())
        .count();
  }

  vehicle_observation observation(
      double const longitude,
      std::int64_t const timestamp,
      std::optional<double> const speed = std::nullopt,
      unsigned const next_stop = 20U,
      std::string status = "IN_TRANSIT_TO") const {
    return {.feed_id_ = "prediction:vp",
            .entity_id_ = "entity",
            .vehicle_id_ = "vehicle",
            .trip_ = {.trip_id_ = "straight",
                      .start_date_ = "20260521",
                      .start_time_ = "01:00:00"},
            .latitude_ = 50.0,
            .longitude_ = longitude,
            .speed_mps_ = speed,
            .current_stop_sequence_ = next_stop,
            .current_status_ = std::move(status),
            .reported_time_ = timestamp,
            .ingested_time_ = timestamp};
  }

  fs::path path_;
  config config_;
  std::optional<data> data_;
};

vehicle_prediction_batch evaluate_at_offset(prediction_fixture& fixture,
                                            std::int64_t const offset) {
  auto const now = fixture.next_scheduled() - 50 + offset;
  auto observations = std::vector<vehicle_observation>{
      fixture.observation(8.004, now - 10), fixture.observation(8.005, now)};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};
  return engine.evaluate(fixture.run(), observations, now);
}

TEST(vehicle_prediction, produces_second_precision_whole_trip_candidates) {
  auto fixture = prediction_fixture{};

  auto const on_time = evaluate_at_offset(fixture, 0);
  auto const early = evaluate_at_offset(fixture, -30);
  auto const late = evaluate_at_offset(fixture, 30);

  ASSERT_TRUE(on_time.eligible());
  ASSERT_GE(on_time.predictions_.size(), 2U);
  auto const first_sequence =
      on_time.predictions_.front().static_stop_sequence_;
  auto const first_arrival =
      std::ranges::find_if(on_time.predictions_, [&](auto const& prediction) {
        return prediction.static_stop_sequence_ == first_sequence &&
               prediction.event_type_ ==
                   vehicle_prediction_event_type::kArrival;
      });
  auto const first_departure =
      std::ranges::find_if(on_time.predictions_, [&](auto const& prediction) {
        return prediction.static_stop_sequence_ == first_sequence &&
               prediction.event_type_ ==
                   vehicle_prediction_event_type::kDeparture;
      });
  EXPECT_NE(first_arrival, end(on_time.predictions_));
  EXPECT_NE(first_departure, end(on_time.predictions_));
  EXPECT_NEAR(*on_time.delay_anchor_seconds_, 0, 2);
  EXPECT_EQ(on_time.transport_, fixture.run().t_);
  for (auto const& prediction : on_time.predictions_) {
    EXPECT_EQ(prediction.predicted_timestamp_seconds_ -
                  prediction.scheduled_timestamp_seconds_,
              prediction.delay_seconds_);
    EXPECT_EQ(prediction.delay_seconds_, *on_time.delay_anchor_seconds_);
  }
  ASSERT_TRUE(early.eligible());
  EXPECT_LT(*early.delay_anchor_seconds_, 0);
  ASSERT_TRUE(late.eligible());
  EXPECT_GT(*late.delay_anchor_seconds_, 0);
}

TEST(vehicle_prediction, observation_age_does_not_shift_moving_eta) {
  auto fixture = prediction_fixture{};
  auto const observed_at = fixture.next_scheduled() - 50;
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.004, observed_at - 10),
      fixture.observation(8.005, observed_at)};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const at_observation =
      engine.evaluate(fixture.run(), observations, observed_at);
  auto const after_feed_pause =
      engine.evaluate(fixture.run(), observations, observed_at + 60);

  ASSERT_TRUE(at_observation.eligible());
  ASSERT_TRUE(after_feed_pause.eligible());
  EXPECT_EQ(at_observation.predictions_.front().predicted_timestamp_seconds_,
            after_feed_pause.predictions_.front().predicted_timestamp_seconds_);
}

TEST(vehicle_prediction, clamps_tolerated_future_observation_to_cycle_time) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() - 50;
  auto prior = fixture.observation(8.004, now + 20);
  prior.ingested_time_ = now - 10;
  auto future = fixture.observation(8.005, now + 30);
  future.ingested_time_ = now;
  auto const observations = std::vector<vehicle_observation>{prior, future};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.candidate_reference_timestamp_seconds_, now);
}

TEST(vehicle_prediction, finds_shape_for_interlined_trip) {
  auto fixture = prediction_fixture{};
  auto const run = fixture.run("continuation", "01:10");
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};
  auto const scheduled =
      std::chrono::duration_cast<std::chrono::seconds>(
          run[1].scheduled_time(n::event_type::kArr).time_since_epoch())
          .count();
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.024, scheduled - 60, std::nullopt, 20U),
      fixture.observation(8.025, scheduled - 50, std::nullopt, 20U)};

  auto const result = engine.evaluate(run, observations, scheduled - 50);

  EXPECT_TRUE(result.eligible());
}

TEST(vehicle_prediction, dwell_anchors_delay_without_inventing_motion) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 45;
  auto observations = std::vector<vehicle_observation>{
      fixture.observation(8.010, now - 10, 0.0, 20U, "STOPPED_AT"),
      fixture.observation(8.010, now, 0.0, 20U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.delay_anchor_seconds_, 45);
  ASSERT_TRUE(result.confidence_.has_value());
  EXPECT_DOUBLE_EQ(result.confidence_->progress_velocity_mps_, 0.0);
}

TEST(vehicle_prediction,
     recovers_after_a_stop_anchor_invalidates_older_section_progress) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 45;
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.015, now - 20, 0.0, 20U, "IN_TRANSIT_TO"),
      fixture.observation(8.010, now - 10, 0.0, 20U, "STOPPED_AT"),
      fixture.observation(8.010, now, 0.0, 20U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.diagnostics_.fresh_observation_count_, 2U);
  EXPECT_EQ(result.delay_anchor_static_stop_sequence_, 20U);
  EXPECT_EQ(result.delay_anchor_seconds_, 45);
}

TEST(vehicle_prediction,
     uses_a_single_verified_stop_anchor_after_incompatible_motion) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 45;
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.015, now - 10, 0.0, 20U, "IN_TRANSIT_TO"),
      fixture.observation(8.010, now, 0.0, 20U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.diagnostics_.fresh_observation_count_, 1U);
  EXPECT_EQ(result.delay_anchor_static_stop_sequence_, 20U);
  EXPECT_EQ(result.delay_anchor_seconds_, 45);
  ASSERT_TRUE(result.confidence_.has_value());
  EXPECT_DOUBLE_EQ(result.confidence_->score_, 1.0);
}

TEST(vehicle_prediction, one_stop_observation_is_still_insufficient_history) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 45;
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.010, now, 0.0, 20U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  EXPECT_EQ(result.diagnostics_.rejection_,
            vehicle_prediction_rejection_reason::kInsufficientHistory);
}

TEST(vehicle_prediction, never_recovers_past_an_invalid_newest_observation) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 45;
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.009, now - 20, std::nullopt, 20U),
      fixture.observation(8.010, now - 10, std::nullopt, 20U),
      fixture.observation(9.000, now, std::nullopt, 20U)};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), observations, now);

  EXPECT_EQ(result.diagnostics_.rejection_,
            vehicle_prediction_rejection_reason::kOffShape);
}

TEST(vehicle_prediction, stopped_at_origin_never_departs_before_schedule) {
  auto fixture = prediction_fixture{};
  auto const run = fixture.run();
  auto const scheduled_departure =
      std::chrono::duration_cast<std::chrono::seconds>(
          run[0].scheduled_time(n::event_type::kDep).time_since_epoch())
          .count();
  auto const now = scheduled_departure - 120;
  auto observations = std::vector<vehicle_observation>{
      fixture.observation(8.000, now - 10, 0.0, 10U, "STOPPED_AT"),
      fixture.observation(8.000, now, 0.0, 10U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(run, observations, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.delay_anchor_static_stop_sequence_, 10U);
  EXPECT_EQ(result.delay_anchor_seconds_, 0);
  ASSERT_FALSE(result.predictions_.empty());
  EXPECT_GE(result.predictions_.front().predicted_timestamp_seconds_,
            scheduled_departure);
}

TEST(vehicle_prediction, reported_speed_conflict_is_diagnostic_only) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() - 50;
  auto const conflicting_speed = std::vector<vehicle_observation>{
      fixture.observation(8.004, now - 10, 40.0),
      fixture.observation(8.005, now, 40.0)};
  auto const no_reported_speed = std::vector<vehicle_observation>{
      fixture.observation(8.004, now - 10), fixture.observation(8.005, now)};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const conflicting =
      engine.evaluate(fixture.run(), conflicting_speed, now);
  auto const baseline = engine.evaluate(fixture.run(), no_reported_speed, now);

  ASSERT_TRUE(conflicting.eligible());
  ASSERT_TRUE(conflicting.confidence_.has_value());
  ASSERT_TRUE(baseline.confidence_.has_value());
  EXPECT_TRUE(conflicting.confidence_->reported_speed_conflict_);
  EXPECT_EQ(conflicting.confidence_->score_, baseline.confidence_->score_);
}

TEST(vehicle_prediction, latest_ingest_corrects_equal_reported_timestamp) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() - 50;
  auto first = fixture.observation(8.004, now - 10);
  auto superseded = fixture.observation(8.005, now);
  auto corrected = fixture.observation(8.006, now);
  corrected.ingested_time_ = now + 1;
  auto const corrected_history =
      std::vector<vehicle_observation>{first, superseded, corrected};
  auto const expected_history =
      std::vector<vehicle_observation>{first, corrected};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result =
      engine.evaluate(fixture.run(), corrected_history, now + 1);
  auto const expected =
      engine.evaluate(fixture.run(), expected_history, now + 1);

  ASSERT_TRUE(result.eligible());
  ASSERT_TRUE(result.implied_progress_m_.has_value());
  ASSERT_TRUE(expected.implied_progress_m_.has_value());
  EXPECT_EQ(result.implied_progress_m_, expected.implied_progress_m_);
  ASSERT_EQ(result.predictions_.size(), expected.predictions_.size());
  EXPECT_EQ(result.predictions_.front().predicted_timestamp_seconds_,
            expected.predictions_.front().predicted_timestamp_seconds_);
}

TEST(vehicle_prediction, rejects_stale_insufficient_and_impossible_history) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled();
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};
  auto stale =
      std::vector<vehicle_observation>{fixture.observation(8.004, now - 401),
                                       fixture.observation(8.005, now - 400)};
  auto insufficient =
      std::vector<vehicle_observation>{fixture.observation(8.005, now)};
  auto impossible = std::vector<vehicle_observation>{
      fixture.observation(8.001, now - 1), fixture.observation(8.009, now)};

  EXPECT_EQ(engine.evaluate(fixture.run(), stale, now).diagnostics_.rejection_,
            vehicle_prediction_rejection_reason::kStaleHistory);
  EXPECT_EQ(
      engine.evaluate(fixture.run(), insufficient, now).diagnostics_.rejection_,
      vehicle_prediction_rejection_reason::kInsufficientHistory);
  EXPECT_EQ(
      engine.evaluate(fixture.run(), impossible, now).diagnostics_.rejection_,
      vehicle_prediction_rejection_reason::kImpossibleSpeed);
}

TEST(vehicle_prediction,
     retains_short_pause_context_but_rejects_long_observation_gaps) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled();
  auto engine = vehicle_prediction_engine{
      *fixture.data_->shapes_,
      vehicle_prediction_policy{.max_observation_age_seconds_ = 300,
                                .max_observation_gap_seconds_ = 120}};
  auto retained_short_pause =
      std::vector<vehicle_observation>{fixture.observation(8.004, now - 310),
                                       fixture.observation(8.005, now - 190)};
  auto stale_latest =
      std::vector<vehicle_observation>{fixture.observation(8.004, now - 410),
                                       fixture.observation(8.005, now - 400)};
  auto long_gap = std::vector<vehicle_observation>{
      fixture.observation(8.003, now - 121), fixture.observation(8.005, now)};
  auto recovered = std::vector<vehicle_observation>{
      fixture.observation(8.003, now - 300),
      fixture.observation(8.004, now - 10), fixture.observation(8.005, now)};

  EXPECT_TRUE(
      engine.evaluate(fixture.run(), retained_short_pause, now).eligible());
  EXPECT_EQ(
      engine.evaluate(fixture.run(), stale_latest, now).diagnostics_.rejection_,
      vehicle_prediction_rejection_reason::kStaleHistory);
  EXPECT_EQ(
      engine.evaluate(fixture.run(), long_gap, now).diagnostics_.rejection_,
      vehicle_prediction_rejection_reason::kInsufficientHistory);
  EXPECT_TRUE(engine.evaluate(fixture.run(), recovered, now).eligible());
}

TEST(vehicle_prediction, rejects_terminal_and_handles_scheduled_short_turn) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 300;
  auto at_terminal = std::vector<vehicle_observation>{
      fixture.observation(8.019, now - 10, 0.0, 30U),
      fixture.observation(8.020, now, 0.0, 30U, "STOPPED_AT")};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  EXPECT_EQ(
      engine.evaluate(fixture.run(), at_terminal, now).diagnostics_.rejection_,
      vehicle_prediction_rejection_reason::kTerminal);

  auto short_turn = fixture.run();
  ++short_turn.stop_range_.from_;
  auto const short_now = fixture.next_scheduled() + 60;
  auto short_history = std::vector<vehicle_observation>{
      fixture.observation(8.011, short_now - 10, std::nullopt, 30U),
      fixture.observation(8.012, short_now, std::nullopt, 30U)};
  auto const result = engine.evaluate(short_turn, short_history, short_now);
  ASSERT_TRUE(result.eligible());
  EXPECT_EQ(result.transport_, short_turn.t_);
  EXPECT_EQ(result.delay_anchor_static_stop_sequence_, 30U);
}

TEST(vehicle_prediction, exports_only_high_confidence_stop_passages) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 10;
  auto certain = std::vector<vehicle_observation>{
      fixture.observation(8.009, now - 10, std::nullopt, 20U),
      fixture.observation(8.011, now, std::nullopt, 30U)};
  auto engine = vehicle_prediction_engine{*fixture.data_->shapes_};

  auto const result = engine.evaluate(fixture.run(), certain, now);

  ASSERT_TRUE(result.eligible());
  ASSERT_EQ(result.observed_passages_.size(), 1U);
  EXPECT_EQ(result.observed_passages_.front().static_stop_sequence_, 20U);
  EXPECT_EQ(result.observed_passages_.front().uncertainty_seconds_, 10);
  EXPECT_EQ(result.diagnostics_.uncertain_passage_count_, 0U);
}

TEST(vehicle_prediction, excludes_uncertain_stop_passages_from_calibration) {
  auto fixture = prediction_fixture{};
  auto const now = fixture.next_scheduled() + 100;
  auto uncertain = std::vector<vehicle_observation>{
      fixture.observation(8.009, now - 100, std::nullopt, 20U),
      fixture.observation(8.011, now, std::nullopt, 30U)};
  auto engine = vehicle_prediction_engine{
      *fixture.data_->shapes_,
      vehicle_prediction_policy{.max_observation_age_seconds_ = 300,
                                .min_observations_ = 2U,
                                .min_progress_velocity_mps_ = 0.01,
                                .max_progress_velocity_mps_ = 55.0,
                                .max_predicted_travel_seconds_ = 4 * 60 * 60,
                                .max_passage_uncertainty_seconds_ = 90}};

  auto const result = engine.evaluate(fixture.run(), uncertain, now);

  ASSERT_TRUE(result.eligible());
  EXPECT_TRUE(result.observed_passages_.empty());
  EXPECT_EQ(result.diagnostics_.uncertain_passage_count_, 1U);
}

TEST(vehicle_prediction_cycle,
     rejects_route_only_unmatched_and_replacement_without_timing_changes) {
  auto fixture = prediction_fixture{};
  fixture.config_.timetable_->vehicle_eta_ = config::timetable::vehicle_eta{
      .mode_ = config::timetable::vehicle_eta::mode::shadow};
  auto const now = fixture.next_scheduled();
  auto position = [&](std::string entity, std::optional<std::string> trip,
                      std::optional<std::string> relationship = std::nullopt) {
    return vehicle_positions::vehicle_position{
        .feed_id_ = "prediction:vp",
        .entity_id_ = std::move(entity),
        .trip_ = {.trip_id_ = std::move(trip),
                  .start_date_ = "20260521",
                  .start_time_ = "01:00:00",
                  .route_id_ = "route",
                  .schedule_relationship_ = std::move(relationship)},
        .reported_position_ = {.pos_ = geo::latlng{50.0, 8.005}},
        .reported_time_ = now,
        .ingested_time_ = now};
  };
  auto positions = vehicle_positions::vehicle_position_store{};
  positions.replace_feed(
      "prediction:vp",
      {position("route-only", std::nullopt), position("unmatched", "unknown"),
       position("replacement", "straight", "REPLACEMENT")});
  auto const scheduled_before =
      fixture.run()[1].scheduled_time(n::event_type::kDep);

  auto const results = evaluate_vehicle_prediction_candidates(
      fixture.config_, *fixture.data_->tags_, *fixture.data_->tt_, nullptr,
      fixture.data_->shapes_.get(), positions, vehicle_observation_history{},
      now);

  ASSERT_EQ(results.size(), 3U);
  auto reasons = std::vector<vehicle_prediction_rejection_reason>{};
  for (auto const& result : results) {
    ASSERT_TRUE(result.batch_.diagnostics_.rejection_.has_value());
    reasons.push_back(*result.batch_.diagnostics_.rejection_);
  }
  EXPECT_NE(std::ranges::find(
                reasons, vehicle_prediction_rejection_reason::kMissingTripId),
            end(reasons));
  EXPECT_NE(std::ranges::find(
                reasons, vehicle_prediction_rejection_reason::kUnresolvedTrip),
            end(reasons));
  EXPECT_NE(
      std::ranges::find(
          reasons,
          vehicle_prediction_rejection_reason::kUnsupportedTripRelationship),
      end(reasons));
  EXPECT_EQ(scheduled_before,
            fixture.run()[1].scheduled_time(n::event_type::kDep));
}

TEST(vehicle_prediction_cycle,
     polling_faster_than_the_feed_does_not_discard_valid_history) {
  auto fixture = prediction_fixture{};
  fixture.config_.timetable_->update_interval_ = 20;
  fixture.config_.timetable_->vehicle_eta_ = config::timetable::vehicle_eta{
      .mode_ = config::timetable::vehicle_eta::mode::shadow};
  auto const now = fixture.next_scheduled();
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.0035, now - 139),
      fixture.observation(8.0040, now - 86), fixture.observation(8.0050, now)};
  auto history = vehicle_observation_history{};
  for (auto const& observation : observations) {
    EXPECT_TRUE(
        history.ingest(observation, {.max_age_ = std::chrono::seconds{300},
                                     .max_observations_per_vehicle_ = 20U}));
  }
  auto positions = vehicle_positions::vehicle_position_store{};
  positions.replace_feed(
      "prediction:vp",
      {{.feed_id_ = "prediction:vp",
        .entity_id_ = "entity",
        .vehicle_ = {.id_ = "vehicle"},
        .trip_ = {.trip_id_ = "straight",
                  .start_date_ = "20260521",
                  .start_time_ = "01:00:00",
                  .route_id_ = "route"},
        .reported_position_ = {.pos_ = geo::latlng{50.0, 8.0050}},
        .current_stop_sequence_ = 20U,
        .current_status_ = "IN_TRANSIT_TO",
        .reported_time_ = now,
        .ingested_time_ = now}});

  auto const results = evaluate_vehicle_prediction_candidates(
      fixture.config_, *fixture.data_->tags_, *fixture.data_->tt_, nullptr,
      fixture.data_->shapes_.get(), positions, history, now);

  auto const direct =
      std::ranges::find(results, vehicle_prediction_context::kDirect,
                        &vehicle_prediction_cycle_result::context_);
  ASSERT_NE(direct, end(results));
  EXPECT_TRUE(direct->batch_.eligible());
  EXPECT_EQ(3U, direct->batch_.diagnostics_.fresh_observation_count_);
}

TEST(vehicle_prediction_cycle,
     provider_candidates_include_only_changed_stop_events) {
  auto fixture = prediction_fixture{};
  fixture.config_.timetable_->vehicle_eta_ = config::timetable::vehicle_eta{
      .mode_ = config::timetable::vehicle_eta::mode::shadow};
  auto const now = fixture.next_scheduled();
  auto history = vehicle_observation_history{};
  for (auto const& observation : {fixture.observation(8.004, now - 10),
                                  fixture.observation(8.005, now)}) {
    EXPECT_TRUE(
        history.ingest(observation, {.max_age_ = std::chrono::seconds{300},
                                     .max_observations_per_vehicle_ = 20U}));
  }
  auto positions = vehicle_positions::vehicle_position_store{};
  positions.replace_feed(
      "prediction:vp",
      {{.feed_id_ = "prediction:vp",
        .entity_id_ = "entity",
        .vehicle_ = {.id_ = "vehicle"},
        .trip_ = {.trip_id_ = "straight",
                  .start_date_ = "20260521",
                  .start_time_ = "01:00:00",
                  .route_id_ = "route"},
        .reported_position_ = {.pos_ = geo::latlng{50.0, 8.005}},
        .current_stop_sequence_ = 20U,
        .current_status_ = "IN_TRANSIT_TO",
        .reported_time_ = now,
        .ingested_time_ = now}});
  auto const run = fixture.run();
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto const rt_transport = rtt.add_rt_transport(
      fixture.data_->tags_->get_src("prediction"), *fixture.data_->tt_, run.t_);
  auto const delayed =
      run[1U].scheduled_time(n::event_type::kArr) + std::chrono::minutes{1};
  rtt.update_time(rt_transport, n::stop_idx_t{1U}, n::event_type::kArr,
                  delayed);

  auto const results = evaluate_vehicle_prediction_candidates(
      fixture.config_, *fixture.data_->tags_, *fixture.data_->tt_, &rtt,
      fixture.data_->shapes_.get(), positions, history, now);

  auto const direct =
      std::ranges::find(results, vehicle_prediction_context::kDirect,
                        &vehicle_prediction_cycle_result::context_);
  ASSERT_NE(direct, end(results));
  ASSERT_TRUE(direct->batch_.eligible());
  ASSERT_EQ(direct->provider_predictions_.size(), 1U);
  EXPECT_EQ(direct->provider_predictions_.front().static_stop_sequence_, 20U);
  EXPECT_EQ(direct->provider_predictions_.front().event_type_,
            vehicle_prediction_event_type::kArrival);
  EXPECT_EQ(direct->provider_predictions_.front().delay_seconds_, 60);
}

TEST(vehicle_prediction_cycle, runtime_control_promotes_statically_off_feed) {
  auto fixture = prediction_fixture{};
  fixture.config_.timetable_->vehicle_eta_ = config::timetable::vehicle_eta{
      .mode_ = config::timetable::vehicle_eta::mode::off};
  auto const now = fixture.next_scheduled();
  auto const observations = std::vector<vehicle_observation>{
      fixture.observation(8.004, now - 10), fixture.observation(8.005, now)};
  auto history = vehicle_observation_history{};
  for (auto const& observation : observations) {
    EXPECT_TRUE(
        history.ingest(observation, {.max_age_ = std::chrono::seconds{300},
                                     .max_observations_per_vehicle_ = 20U}));
  }
  auto positions = vehicle_positions::vehicle_position_store{};
  positions.replace_feed(
      "prediction:vp",
      {{.feed_id_ = "prediction:vp",
        .entity_id_ = "entity",
        .vehicle_ = {.id_ = "vehicle"},
        .trip_ = {.trip_id_ = "straight",
                  .start_date_ = "20260521",
                  .start_time_ = "01:00:00",
                  .route_id_ = "route"},
        .reported_position_ = {.pos_ = geo::latlng{50.0, 8.005}},
        .current_stop_sequence_ = 20U,
        .current_status_ = "IN_TRANSIT_TO",
        .reported_time_ = now,
        .ingested_time_ = now}});
  auto const control_path =
      fs::temp_directory_path() / "motis-eta-runtime-promotion.json";
  {
    auto output = std::ofstream{control_path};
    output
        << R"({"version":1,"forceOff":false,"feeds":[{"feedId":"prediction","state":"effective"}]})";
  }
  auto control = vehicle_eta_runtime_control{control_path};
  ASSERT_TRUE(control.reload());

  auto const results = evaluate_vehicle_prediction_candidates(
      fixture.config_, *fixture.data_->tags_, *fixture.data_->tt_, nullptr,
      fixture.data_->shapes_.get(), positions, history, now, &control);

  auto const direct =
      std::ranges::find(results, vehicle_prediction_context::kDirect,
                        &vehicle_prediction_cycle_result::context_);
  ASSERT_NE(direct, end(results));
  EXPECT_TRUE(direct->batch_.eligible());
  auto ec = std::error_code{};
  fs::remove(control_path, ec);
}

TEST(vehicle_prediction_continuation,
     derives_only_the_adjacent_block_leg_and_absorbs_layover) {
  auto fixture = prediction_fixture{};
  auto const current = fixture.run();
  auto const terminal_scheduled =
      to_seconds(current[current.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto batch = vehicle_prediction_batch{
      .transport_ = current.t_,
      .candidate_reference_timestamp_seconds_ = terminal_scheduled - 60,
      .predictions_ = {{.static_stop_sequence_ = 30U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = terminal_scheduled,
                        .predicted_timestamp_seconds_ =
                            terminal_scheduled + 180,
                        .delay_seconds_ = 180}},
      .confidence_ = vehicle_prediction_confidence{.score_ = 0.8}};
  auto const current_trip_id = fixture.data_->tags_->id(
      *fixture.data_->tt_, current[0], n::event_type::kDep);

  auto const continuation = derive_vehicle_prediction_continuation(
      *fixture.data_->tt_, *fixture.data_->tags_, current, "prediction",
      current_trip_id, batch);

  ASSERT_TRUE(continuation.has_value());
  EXPECT_NE(continuation->trip_id_.find("continuation"), std::string::npos);
  EXPECT_EQ(continuation->trip_stop_range_.from_, 2U);
  EXPECT_EQ(continuation->trip_stop_range_.to_, 5U);
  EXPECT_EQ(continuation->provenance_.incoming_trip_id_, current_trip_id);
  EXPECT_EQ(continuation->provenance_.route_id_, "route");
  EXPECT_EQ(continuation->provenance_.route_short_name_, "1");
  EXPECT_EQ(continuation->provenance_.headsign_, "Incoming");
  EXPECT_EQ(continuation->provenance_.propagated_delay_seconds_, 180);
  EXPECT_EQ(continuation->provenance_.expected_terminal_arrival_seconds_,
            terminal_scheduled + 180);
  EXPECT_TRUE(std::ranges::all_of(
      continuation->predictions_,
      [](auto const& prediction) { return prediction.delay_seconds_ == 180; }));
  ASSERT_EQ(4U, continuation->predictions_.size());
  EXPECT_TRUE(std::ranges::none_of(
      continuation->predictions_, [](auto const& prediction) {
        return (prediction.static_stop_sequence_ == 10U &&
                prediction.event_type_ ==
                    vehicle_prediction_event_type::kArrival) ||
               (prediction.static_stop_sequence_ == 30U &&
                prediction.event_type_ ==
                    vehicle_prediction_event_type::kDeparture);
      }));

  batch.predictions_.front().predicted_timestamp_seconds_ =
      terminal_scheduled - 60;
  auto const absorbed = derive_vehicle_prediction_continuation(
      *fixture.data_->tt_, *fixture.data_->tags_, current, "prediction",
      current_trip_id, batch);
  ASSERT_TRUE(absorbed.has_value());
  EXPECT_EQ(absorbed->provenance_.propagated_delay_seconds_, 0);

  auto const middle_leg = fixture.run("continuation", "01:10");
  auto const middle_terminal =
      to_seconds(middle_leg[middle_leg.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto middle_batch = batch;
  middle_batch.predictions_.front().static_stop_sequence_ = 30U;
  middle_batch.predictions_.front().scheduled_timestamp_seconds_ =
      middle_terminal;
  middle_batch.predictions_.front().predicted_timestamp_seconds_ =
      middle_terminal + 120;
  middle_batch.predictions_.front().delay_seconds_ = 120;
  auto const middle_trip_id = fixture.data_->tags_->id(
      *fixture.data_->tt_, middle_leg[0U], n::event_type::kDep);
  auto const third = derive_vehicle_prediction_continuation(
      *fixture.data_->tt_, *fixture.data_->tags_, middle_leg, "prediction",
      middle_trip_id, middle_batch);
  ASSERT_TRUE(third.has_value());
  EXPECT_NE(third->trip_id_.find("third"), std::string::npos);

  auto const last_leg = fixture.run("third", "01:20");
  EXPECT_FALSE(derive_vehicle_prediction_continuation(
                   *fixture.data_->tt_, *fixture.data_->tags_, last_leg,
                   "prediction", third->trip_id_, middle_batch)
                   .has_value());
}

TEST(vehicle_prediction_continuation,
     retains_until_direct_gps_is_eligible_and_expires_after_freshness) {
  auto fixture = prediction_fixture{};
  auto const current = fixture.run();
  auto const terminal_scheduled =
      to_seconds(current[current.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto const reference = terminal_scheduled - 60;
  auto const batch = vehicle_prediction_batch{
      .transport_ = current.t_,
      .candidate_reference_timestamp_seconds_ = reference,
      .predictions_ = {{.static_stop_sequence_ = 30U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = terminal_scheduled,
                        .predicted_timestamp_seconds_ =
                            terminal_scheduled + 180,
                        .delay_seconds_ = 180}},
      .confidence_ = vehicle_prediction_confidence{.score_ = 0.8}};
  auto const current_trip_id = fixture.data_->tags_->id(
      *fixture.data_->tt_, current[0], n::event_type::kDep);
  auto const continuation = derive_vehicle_prediction_continuation(
      *fixture.data_->tt_, *fixture.data_->tags_, current, "prediction",
      current_trip_id, batch);
  ASSERT_TRUE(continuation.has_value());
  auto entries = std::vector<vehicle_prediction_diagnostic_entry>{};
  for (auto const& event : continuation->predictions_) {
    entries.push_back(
        {.transport_ = continuation->transport_,
         .static_stop_sequence_ = event.static_stop_sequence_,
         .event_type_ = event.event_type_,
         .trip_id_ = continuation->trip_id_,
         .trip_stop_range_ = continuation->trip_stop_range_,
         .observed_at_seconds_ = reference,
         .scheduled_timestamp_seconds_ = event.scheduled_timestamp_seconds_,
         .gps_ =
             prediction_candidate_diagnostic{
                 .source_ = vehicle_prediction_source::kGps,
                 .predicted_timestamp_seconds_ =
                     event.predicted_timestamp_seconds_,
                 .delay_seconds_ = event.delay_seconds_,
                 .confidence_ = continuation->confidence_,
                 .reference_timestamp_seconds_ = reference},
         .effective_ = {.source_ = vehicle_prediction_source::kGps,
                        .predicted_timestamp_seconds_ =
                            event.predicted_timestamp_seconds_,
                        .delay_seconds_ = event.delay_seconds_},
         .selected_source_ = vehicle_prediction_source::kGps,
         .gps_context_ = vehicle_prediction_context::kIncomingBlockLeg,
         .context_ = vehicle_prediction_context::kIncomingBlockLeg,
         .incoming_leg_provenance_ = continuation->provenance_});
  }
  auto const store = vehicle_prediction_diagnostics_store::build(
      true, std::move(entries), reference);
  ASSERT_NE(nullptr, store);

  auto rejected =
      vehicle_prediction_cycle_result{.trip_id_ = continuation->trip_id_};
  rejected.batch_.transport_ = continuation->transport_;
  rejected.batch_.diagnostics_.rejection_ =
      vehicle_prediction_rejection_reason::kInsufficientHistory;
  auto at_boundary = merge_retained_vehicle_prediction_continuations(
      store.get(), {rejected}, reference + 300, 300);
  ASSERT_EQ(1U, at_boundary.size());
  EXPECT_EQ(vehicle_prediction_context::kIncomingBlockLeg,
            at_boundary.front().context_);
  EXPECT_TRUE(at_boundary.front().batch_.eligible());

  auto fresh_incoming = at_boundary.front();
  fresh_incoming.batch_.candidate_reference_timestamp_seconds_ = reference + 40;
  for (auto& event : fresh_incoming.batch_.predictions_) {
    event.predicted_timestamp_seconds_ += 60;
    event.delay_seconds_ += 60;
  }
  auto incoming_selected = merge_retained_vehicle_prediction_continuations(
      store.get(), {fresh_incoming}, reference + 40, 300);
  ASSERT_EQ(1U, incoming_selected.size());
  EXPECT_EQ(
      reference + 40,
      incoming_selected.front().batch_.candidate_reference_timestamp_seconds_);
  EXPECT_EQ(
      240,
      incoming_selected.front().batch_.predictions_.front().delay_seconds_);

  auto direct = rejected;
  direct.batch_.candidate_reference_timestamp_seconds_ = reference + 20;
  direct.batch_.confidence_ = vehicle_prediction_confidence{.score_ = 0.9};
  direct.batch_.predictions_ = continuation->predictions_;
  direct.batch_.diagnostics_.rejection_ = std::nullopt;
  auto both_current = merge_retained_vehicle_prediction_continuations(
      store.get(), {fresh_incoming, direct}, reference + 40, 300);
  ASSERT_EQ(1U, both_current.size());
  EXPECT_EQ(vehicle_prediction_context::kDirect, both_current.front().context_);
  EXPECT_EQ(reference + 20,
            both_current.front().batch_.candidate_reference_timestamp_seconds_);
  auto direct_selected = merge_retained_vehicle_prediction_continuations(
      store.get(), {direct}, reference + 20, 300);
  ASSERT_EQ(1U, direct_selected.size());
  EXPECT_EQ(vehicle_prediction_context::kDirect,
            direct_selected.front().context_);
  EXPECT_EQ(
      reference + 20,
      direct_selected.front().batch_.candidate_reference_timestamp_seconds_);

  auto expired = merge_retained_vehicle_prediction_continuations(
      store.get(), {rejected}, reference + 301, 300);
  ASSERT_EQ(1U, expired.size());
  EXPECT_FALSE(expired.front().batch_.eligible());
  EXPECT_EQ(vehicle_prediction_context::kDirect, expired.front().context_);

  for (auto& entry : store->entries_) {
    entry.selected_source_ = vehicle_prediction_source::kProvider;
    entry.context_ = vehicle_prediction_context::kDirect;
    entry.effective_ = {
        .source_ = vehicle_prediction_source::kProvider,
        .predicted_timestamp_seconds_ = entry.scheduled_timestamp_seconds_,
        .delay_seconds_ = 0};
  }
  EXPECT_EQ(nullptr, store->find_incoming_leg(continuation->transport_,
                                              continuation->trip_id_));
  auto after_provider_disappears =
      merge_retained_vehicle_prediction_continuations(store.get(), {rejected},
                                                      reference + 20, 300);
  ASSERT_EQ(1U, after_provider_disappears.size());
  ASSERT_TRUE(after_provider_disappears.front().batch_.eligible());
  EXPECT_EQ(vehicle_prediction_context::kIncomingBlockLeg,
            after_provider_disappears.front().context_);
  auto const restored = select_vehicle_prediction_source(
      {.transport_ = continuation->transport_,
       .now_seconds_ = reference + 20,
       .gps_ =
           timing_source_candidate{
               .source_ = vehicle_prediction_source::kGps,
               .reference_timestamp_seconds_ = reference,
               .confidence_ = continuation->confidence_,
               .physically_reachable_ = true,
               .predictions_ =
                   after_provider_disappears.front().batch_.predictions_}},
      {.max_candidate_age_seconds_ = 300,
       .max_timestamp_skew_seconds_ = 60,
       .max_progress_difference_m_ = 100.0,
       .min_provider_confidence_ = 0.5,
       .min_gps_confidence_ = 0.5,
       .min_source_switch_confidence_advantage_ = 0.1,
       .state_ttl_seconds_ = 300,
       .flap_window_seconds_ = 120,
       .minute_boundary_hysteresis_seconds_ = 10});
  EXPECT_EQ(vehicle_prediction_source::kGps, restored.source_);
}

TEST(vehicle_prediction_continuation,
     applies_only_to_the_next_trip_range_on_the_shared_transport) {
  auto fixture = prediction_fixture{};
  auto const current = fixture.run("continuation", "01:10");
  auto const terminal_scheduled =
      to_seconds(current[current.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto const batch = vehicle_prediction_batch{
      .transport_ = current.t_,
      .candidate_reference_timestamp_seconds_ = terminal_scheduled - 60,
      .predictions_ = {{.static_stop_sequence_ = 30U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = terminal_scheduled,
                        .predicted_timestamp_seconds_ =
                            terminal_scheduled + 180,
                        .delay_seconds_ = 180}},
      .confidence_ = vehicle_prediction_confidence{.score_ = 0.8}};
  auto const current_trip_id = fixture.data_->tags_->id(
      *fixture.data_->tt_, current[0], n::event_type::kDep);
  auto const continuation = derive_vehicle_prediction_continuation(
      *fixture.data_->tt_, *fixture.data_->tags_, current, "prediction",
      current_trip_id, batch);
  ASSERT_TRUE(continuation.has_value());
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto const before =
      n::rt::frun::from_t(*fixture.data_->tt_, &rtt, current.t_);
  auto const prior_leg_time = before[3].time(n::event_type::kArr);
  auto const next_leg_time = before[5].time(n::event_type::kArr);
  auto state = vehicle_prediction_overlay_state{};
  auto const selected = selected_vehicle_prediction_trip{
      .transport_ = continuation->transport_,
      .source_ = fixture.data_->tags_->get_src("prediction"),
      .trip_stop_range_ = continuation->trip_stop_range_,
      .predictions_ = continuation->predictions_};

  auto const applied = apply_vehicle_prediction_overlay(
      *fixture.data_->tt_, rtt, {&selected, 1U}, {}, state);

  EXPECT_EQ(applied.applied_trips_, 1U);
  EXPECT_EQ(applied.applied_transports_,
            std::vector<n::transport>{continuation->transport_});
  auto const after = n::rt::frun::from_t(*fixture.data_->tt_, &rtt, current.t_);
  EXPECT_EQ(after[3].time(n::event_type::kArr), prior_leg_time);
  EXPECT_EQ(after[5].time(n::event_type::kArr),
            next_leg_time + std::chrono::minutes{3});
}

TEST(vehicle_prediction_overlay,
     rejects_interlined_boundary_atomically_after_deadband_rounding) {
  auto fixture = prediction_fixture{};
  auto const incoming = fixture.run();
  auto const outgoing = fixture.run("continuation", "01:10");
  ASSERT_EQ(incoming.t_, outgoing.t_);
  auto const incoming_arrival =
      to_seconds(incoming[incoming.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto const outgoing_departure =
      to_seconds(outgoing[0U].scheduled_time(n::event_type::kDep));
  ASSERT_EQ(incoming_arrival, outgoing_departure);
  auto const source = fixture.data_->tags_->get_src("prediction");
  auto const selected = std::array{
      selected_vehicle_prediction_trip{
          .transport_ = incoming.t_,
          .source_ = source,
          .trip_stop_range_ = incoming.stop_range_,
          .predictions_ = {{.static_stop_sequence_ = 30U,
                            .event_type_ =
                                vehicle_prediction_event_type::kArrival,
                            .scheduled_timestamp_seconds_ = incoming_arrival,
                            .predicted_timestamp_seconds_ =
                                incoming_arrival + 89,
                            .delay_seconds_ = 89}}},
      selected_vehicle_prediction_trip{
          .transport_ = outgoing.t_,
          .source_ = source,
          .trip_stop_range_ = outgoing.stop_range_,
          .predictions_ = {
              {.static_stop_sequence_ = 10U,
               .event_type_ = vehicle_prediction_event_type::kDeparture,
               .scheduled_timestamp_seconds_ = outgoing_departure,
               .predicted_timestamp_seconds_ = outgoing_departure + 60,
               .delay_seconds_ = 60}}}};
  auto state = vehicle_prediction_overlay_state{
      .rendered_events_ = {
          {.transport_ = incoming.t_,
           .trip_stop_range_ = incoming.stop_range_,
           .static_stop_sequence_ = 30U,
           .event_type_ = vehicle_prediction_event_type::kArrival,
           .delay_minutes_ = 2}}};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto const before =
      n::rt::frun::from_t(*fixture.data_->tt_, &rtt, incoming.t_);
  auto const before_arrival = before[2U].time(n::event_type::kArr);
  auto const before_departure = before[2U].time(n::event_type::kDep);

  auto const result = apply_vehicle_prediction_overlay(*fixture.data_->tt_, rtt,
                                                       selected, {}, state);

  EXPECT_EQ(0U, result.applied_trips_);
  EXPECT_EQ(2U, result.rejected_trips_);
  EXPECT_TRUE(result.applied_transports_.empty());
  auto const after =
      n::rt::frun::from_t(*fixture.data_->tt_, &rtt, incoming.t_);
  EXPECT_EQ(before_arrival, after[2U].time(n::event_type::kArr));
  EXPECT_EQ(before_departure, after[2U].time(n::event_type::kDep));

  auto provider_rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto const rt_transport =
      provider_rtt.add_rt_transport(source, *fixture.data_->tt_, incoming.t_);
  auto const delayed_incoming =
      n::unixtime_t{std::chrono::duration_cast<n::unixtime_t::duration>(
          std::chrono::seconds{incoming_arrival + 120})};
  provider_rtt.update_time(rt_transport, n::stop_idx_t{2U}, n::event_type::kArr,
                           delayed_incoming);
  auto single_state = vehicle_prediction_overlay_state{};
  auto const single = apply_vehicle_prediction_overlay(
      *fixture.data_->tt_, provider_rtt, {&selected[1U], 1U}, {}, single_state);
  EXPECT_EQ(0U, single.applied_trips_);
  EXPECT_EQ(1U, single.rejected_trips_);
  EXPECT_TRUE(single.applied_transports_.empty());
  auto const provider_after =
      n::rt::frun::from_t(*fixture.data_->tt_, &provider_rtt, incoming.t_);
  EXPECT_EQ(delayed_incoming, provider_after[2U].time(n::event_type::kArr));
  EXPECT_EQ(before_departure, provider_after[2U].time(n::event_type::kDep));
  EXPECT_TRUE(single_state.rendered_events_.empty());
}

TEST(vehicle_prediction_overlay,
     restores_downstream_provider_segment_after_incoming_gps_dispatch) {
  auto fixture = prediction_fixture{};
  auto const incoming = fixture.run();
  auto const outgoing = fixture.run("continuation", "01:10");
  auto const boundary =
      to_seconds(incoming[incoming.stop_range_.size() - 1U].scheduled_time(
          n::event_type::kArr));
  auto const source = fixture.data_->tags_->get_src("prediction");
  auto const selected = std::array{
      selected_vehicle_prediction_trip{
          .transport_ = outgoing.t_,
          .source_ = source,
          .trip_stop_range_ = outgoing.stop_range_,
          .predictions_ = {{.static_stop_sequence_ = 10U,
                            .event_type_ =
                                vehicle_prediction_event_type::kDeparture,
                            .scheduled_timestamp_seconds_ = boundary,
                            .predicted_timestamp_seconds_ = boundary + 120,
                            .delay_seconds_ = 120}}},
      selected_vehicle_prediction_trip{
          .transport_ = incoming.t_,
          .source_ = source,
          .trip_stop_range_ = incoming.stop_range_,
          .predictions_ = {
              {.static_stop_sequence_ = 30U,
               .event_type_ = vehicle_prediction_event_type::kArrival,
               .scheduled_timestamp_seconds_ = boundary,
               .predicted_timestamp_seconds_ = boundary + 60,
               .delay_seconds_ = 60}}}};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto state = vehicle_prediction_overlay_state{};

  auto const result = apply_vehicle_prediction_overlay(*fixture.data_->tt_, rtt,
                                                       selected, {}, state);

  ASSERT_EQ(result.applied_trips_, 2U);
  auto const updated =
      n::rt::frun::from_t(*fixture.data_->tt_, &rtt, outgoing.t_);
  EXPECT_EQ(
      to_seconds(updated[outgoing.stop_range_.from_].time(n::event_type::kDep)),
      boundary + 120);
}

TEST(vehicle_prediction_overlay, reports_clamped_early_departure) {
  auto fixture = prediction_fixture{};
  auto const run = fixture.run();
  auto const scheduled_departure =
      to_seconds(run[0U].scheduled_time(n::event_type::kDep));
  auto const raw_prediction = scheduled_departure - 120;
  auto const trip = selected_vehicle_prediction_trip{
      .transport_ = run.t_,
      .source_ = fixture.data_->tags_->get_src("prediction"),
      .predictions_ = {
          {.static_stop_sequence_ = 10U,
           .event_type_ = vehicle_prediction_event_type::kDeparture,
           .scheduled_timestamp_seconds_ = scheduled_departure,
           .predicted_timestamp_seconds_ = raw_prediction,
           .delay_seconds_ = -120}}};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto state = vehicle_prediction_overlay_state{};

  auto const result = apply_vehicle_prediction_overlay(
      *fixture.data_->tt_, rtt, {&trip, 1U},
      {.early_departure_tolerance_seconds_ = 60}, state);

  ASSERT_EQ(result.applied_trips_, 1U);
  ASSERT_EQ(state.rendered_events_.size(), 1U);
  EXPECT_EQ(state.rendered_events_.front().effective_timestamp_seconds_,
            scheduled_departure - 60);
  EXPECT_EQ(trip.predictions_.front().predicted_timestamp_seconds_,
            raw_prediction);
}

TEST(vehicle_prediction_overlay, rejects_timestamp_recomposition_overflow) {
  auto fixture = prediction_fixture{};
  auto const run = fixture.run();
  auto const scheduled =
      to_seconds(run[1U].scheduled_time(n::event_type::kArr));
  auto const trip = selected_vehicle_prediction_trip{
      .transport_ = run.t_,
      .source_ = fixture.data_->tags_->get_src("prediction"),
      .predictions_ = {
          {.static_stop_sequence_ = 20U,
           .event_type_ = vehicle_prediction_event_type::kArrival,
           .scheduled_timestamp_seconds_ = scheduled,
           .predicted_timestamp_seconds_ =
               std::numeric_limits<std::int64_t>::max(),
           .delay_seconds_ = std::numeric_limits<std::int64_t>::max() -
                             scheduled}}};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto state = vehicle_prediction_overlay_state{};

  auto const result = apply_vehicle_prediction_overlay(
      *fixture.data_->tt_, rtt, {&trip, 1U}, {}, state);

  EXPECT_EQ(result.applied_trips_, 0U);
  EXPECT_EQ(result.rejected_trips_, 1U);
  EXPECT_TRUE(state.rendered_events_.empty());
}

TEST(vehicle_prediction_overlay,
     rounds_subminute_early_departure_clamp_toward_schedule) {
  auto fixture = prediction_fixture{};
  auto const run = fixture.run();
  auto const scheduled_departure =
      to_seconds(run[0U].scheduled_time(n::event_type::kDep));
  auto const trip = selected_vehicle_prediction_trip{
      .transport_ = run.t_,
      .source_ = fixture.data_->tags_->get_src("prediction"),
      .predictions_ = {
          {.static_stop_sequence_ = 10U,
           .event_type_ = vehicle_prediction_event_type::kDeparture,
           .scheduled_timestamp_seconds_ = scheduled_departure,
           .predicted_timestamp_seconds_ = scheduled_departure - 120,
           .delay_seconds_ = -120}}};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto state = vehicle_prediction_overlay_state{};

  auto const result = apply_vehicle_prediction_overlay(
      *fixture.data_->tt_, rtt, {&trip, 1U},
      {.early_departure_tolerance_seconds_ = 30}, state);

  ASSERT_EQ(result.applied_trips_, 1U);
  auto const updated = n::rt::frun::from_t(*fixture.data_->tt_, &rtt, run.t_);
  EXPECT_EQ(to_seconds(updated[0U].time(n::event_type::kDep)),
            scheduled_departure);
  ASSERT_EQ(state.rendered_events_.size(), 1U);
  EXPECT_EQ(state.rendered_events_.front().effective_timestamp_seconds_,
            scheduled_departure);
  EXPECT_EQ(state.rendered_events_.front().delay_minutes_, 0);
}

TEST(
    vehicle_prediction_overlay,
    applies_from_provider_baseline_without_compounding_and_rejects_atomically) {
  auto fixture = prediction_fixture{};
  auto const scheduled_run = fixture.run();
  auto const transport = scheduled_run.t_;
  auto const scheduled_arrival =
      std::chrono::duration_cast<std::chrono::seconds>(
          scheduled_run[1]
              .scheduled_time(n::event_type::kArr)
              .time_since_epoch())
          .count();
  auto const scheduled_departure =
      std::chrono::duration_cast<std::chrono::seconds>(
          scheduled_run[1]
              .scheduled_time(n::event_type::kDep)
              .time_since_epoch())
          .count();
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto const provider =
      n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto effective = provider;
  auto state = vehicle_prediction_overlay_state{};
  auto const trip = selected_vehicle_prediction_trip{
      .transport_ = transport,
      .source_ = fixture.data_->tags_->get_src("prediction"),
      .predictions_ = {
          {.static_stop_sequence_ = 20U,
           .event_type_ = vehicle_prediction_event_type::kArrival,
           .scheduled_timestamp_seconds_ = scheduled_arrival,
           .predicted_timestamp_seconds_ = scheduled_arrival + 120,
           .delay_seconds_ = 120},
          {.static_stop_sequence_ = 20U,
           .event_type_ = vehicle_prediction_event_type::kDeparture,
           .scheduled_timestamp_seconds_ = scheduled_departure,
           .predicted_timestamp_seconds_ = scheduled_departure + 120,
           .delay_seconds_ = 120}}};

  EXPECT_EQ(apply_vehicle_prediction_overlay(*fixture.data_->tt_, effective,
                                             {&trip, 1U}, {}, state)
                .applied_trips_,
            1U);
  auto const provider_run =
      n::rt::frun::from_t(*fixture.data_->tt_, &provider, transport);
  auto const effective_run =
      n::rt::frun::from_t(*fixture.data_->tt_, &effective, transport);
  EXPECT_EQ(provider_run[1].time(n::event_type::kArr),
            provider_run[1].scheduled_time(n::event_type::kArr));
  EXPECT_EQ(effective_run[1].time(n::event_type::kArr),
            provider_run[1].scheduled_time(n::event_type::kArr) +
                std::chrono::minutes{2});

  auto const rt_transport = effective.resolve_rt(transport);
  ASSERT_NE(n::rt_transport_idx_t::invalid(), rt_transport);
  effective.rt_transport_is_cancelled_.set(to_idx(rt_transport), true);
  effective.rt_transport_location_seq_[rt_transport][1U] =
      n::stop{scheduled_run[2U].get_location_idx(), false, false, false, false}
          .value();
  auto const operational_stop =
      effective.rt_transport_location_seq_[rt_transport][1U];

  EXPECT_EQ(apply_vehicle_prediction_overlay(*fixture.data_->tt_, effective,
                                             {&trip, 1U}, {}, state)
                .applied_trips_,
            1U);
  EXPECT_TRUE(effective.rt_transport_is_cancelled_[to_idx(rt_transport)]);
  EXPECT_EQ(operational_stop,
            effective.rt_transport_location_seq_[rt_transport][1U]);

  auto next_cycle = provider;
  EXPECT_EQ(apply_vehicle_prediction_overlay(*fixture.data_->tt_, next_cycle,
                                             {&trip, 1U}, {}, state)
                .applied_trips_,
            1U);
  EXPECT_EQ(
      n::rt::frun::from_t(*fixture.data_->tt_, &next_cycle, transport)[1].time(
          n::event_type::kArr),
      effective_run[1].time(n::event_type::kArr));

  auto invalid = trip;
  invalid.predictions_.push_back(
      {.static_stop_sequence_ = 999U,
       .event_type_ = vehicle_prediction_event_type::kArrival,
       .scheduled_timestamp_seconds_ = scheduled_arrival,
       .predicted_timestamp_seconds_ = scheduled_arrival + 180,
       .delay_seconds_ = 180});
  auto rejected = provider;
  EXPECT_EQ(apply_vehicle_prediction_overlay(*fixture.data_->tt_, rejected,
                                             {&invalid, 1U}, {}, state)
                .rejected_trips_,
            1U);
  EXPECT_EQ(
      n::rt::frun::from_t(*fixture.data_->tt_, &rejected, transport)[1].time(
          n::event_type::kArr),
      provider_run[1].time(n::event_type::kArr));
}

TEST(vehicle_prediction_overlay, prunes_hysteresis_for_absent_transports) {
  auto fixture = prediction_fixture{};
  auto const base_day = date::sys_days{date::year{2026} / 5 / 21};
  auto rtt = n::rt::create_rt_timetable(*fixture.data_->tt_, base_day);
  auto state = vehicle_prediction_overlay_state{
      .rendered_events_ = {{.transport_ = fixture.run().t_,
                            .static_stop_sequence_ = 20U,
                            .delay_minutes_ = 2}}};

  auto const result =
      apply_vehicle_prediction_overlay(*fixture.data_->tt_, rtt, {}, {}, state);

  EXPECT_EQ(0U, result.applied_trips_);
  EXPECT_EQ(0U, result.rejected_trips_);
  EXPECT_TRUE(state.rendered_events_.empty());
}

}  // namespace
}  // namespace motis
