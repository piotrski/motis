#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <iterator>

#include "boost/json.hpp"

#include "motis/rt/vehicle_eta_calibration.h"

namespace motis {
namespace {

namespace fs = std::filesystem;

vehicle_prediction_cycle_result candidate(std::int64_t const scheduled) {
  auto result = vehicle_prediction_cycle_result{
      .feed_ = "A", .trip_id_ = "service-trip", .mode_ = nigiri::clasz::kBus};
  result.batch_.transport_ =
      nigiri::transport{nigiri::transport_idx_t{1U}, nigiri::day_idx_t{2U}};
  result.batch_.predictions_ = {
      {.static_stop_sequence_ = 4U,
       .event_type_ = vehicle_prediction_event_type::kArrival,
       .scheduled_timestamp_seconds_ = scheduled,
       .predicted_timestamp_seconds_ = scheduled + 30,
       .delay_seconds_ = 30,
       .horizon_seconds_ = 630}};
  result.batch_.confidence_ = vehicle_prediction_confidence{.score_ = 0.8};
  result.provider_predictions_ = {
      {.static_stop_sequence_ = 4U,
       .event_type_ = vehicle_prediction_event_type::kArrival,
       .scheduled_timestamp_seconds_ = scheduled,
       .predicted_timestamp_seconds_ = scheduled + 90,
       .delay_seconds_ = 90}};
  return result;
}

TEST(vehicle_eta_calibration,
     captures_once_without_lookahead_and_scores_passage) {
  auto const dir = fs::temp_directory_path() / "motis-eta-calibration-test";
  fs::remove_all(dir);
  auto store = vehicle_eta_calibration{dir};
  auto value = candidate(1'600);
  auto const captured = store.ingest({&value, 1U}, 1'000, 60);
  EXPECT_EQ(1U, captured.pending_entries_);
  EXPECT_EQ(0U, captured.completed_entries_);
  EXPECT_EQ(0U, captured.queued_records_);
  EXPECT_EQ(0U, captured.queued_bytes_);
  static_cast<void>(store.ingest({&value, 1U}, 1'010, 60));
  EXPECT_FALSE(fs::exists(dir / "calibration.jsonl"));

  value.batch_.observed_passages_ = {{.static_stop_sequence_ = 4U,
                                      .observed_timestamp_seconds_ = 1'640,
                                      .uncertainty_seconds_ = 20}};
  auto const completed = store.ingest({&value, 1U}, 1'650, 60);
  EXPECT_EQ(0U, completed.pending_entries_);
  EXPECT_EQ(1U, completed.completed_entries_);
  EXPECT_EQ(1U, completed.queued_records_);
  EXPECT_GT(completed.queued_bytes_, 0U);
  EXPECT_EQ(0U, completed.dropped_records_);
  store.flush();
  auto const records =
      vehicle_eta_calibration::read_records(dir / "calibration.jsonl");
  ASSERT_EQ(1U, records.size());
  auto const& record = records.front().as_object();
  EXPECT_EQ(1'000, record.at("capturedTime").as_int64());
  EXPECT_EQ(-10, record.at("gpsErrorSeconds").as_int64());
  EXPECT_EQ(50, record.at("providerErrorSeconds").as_int64());
  EXPECT_FALSE(record.contains("vehicleId"));
  EXPECT_FALSE(record.contains("latitude"));
  EXPECT_FALSE(fs::exists(dir / "report.json"));

  store.generate_report();
  auto report_input = std::ifstream{dir / "report.json"};
  auto const report_text =
      std::string{std::istreambuf_iterator<char>{report_input},
                  std::istreambuf_iterator<char>{}};
  auto const report = boost::json::parse(report_text);
  auto const& group =
      report.as_object().at("groups").as_array().front().as_object();
  EXPECT_TRUE(group.contains("gpsMedianAbsoluteError95CiSeconds"));
  EXPECT_TRUE(group.contains("providerMedianAbsoluteError95CiSeconds"));
  fs::remove_all(dir);
}

TEST(vehicle_eta_calibration,
     scores_passage_after_forecast_leaves_prediction_horizon) {
  auto const dir =
      fs::temp_directory_path() / "motis-eta-calibration-passed-stop-test";
  fs::remove_all(dir);
  auto store = vehicle_eta_calibration{dir};
  auto value = candidate(1'600);
  static_cast<void>(store.ingest({&value, 1U}, 1'000, 60));

  value.batch_.predictions_.front().static_stop_sequence_ = 5U;
  value.batch_.predictions_.front().scheduled_timestamp_seconds_ = 1'900;
  value.batch_.predictions_.front().predicted_timestamp_seconds_ = 1'930;
  value.batch_.observed_passages_ = {{.static_stop_sequence_ = 4U,
                                      .observed_timestamp_seconds_ = 1'640,
                                      .uncertainty_seconds_ = 20}};

  auto const completed = store.ingest({&value, 1U}, 1'650, 60);

  EXPECT_EQ(1U, completed.pending_entries_);
  EXPECT_EQ(1U, completed.completed_entries_);
  EXPECT_EQ(1U, completed.queued_records_);
  store.flush();
  EXPECT_EQ(
      1U,
      vehicle_eta_calibration::read_records(dir / "calibration.jsonl").size());
  fs::remove_all(dir);
}

TEST(vehicle_eta_calibration, ignores_partial_final_jsonl_record) {
  auto const path =
      fs::temp_directory_path() / "motis-eta-calibration-partial.jsonl";
  {
    auto output = std::ofstream{path};
    output << "{\"version\":1}\n{\"partial\":";
  }
  auto const records = vehicle_eta_calibration::read_records(path);
  ASSERT_EQ(1U, records.size());
  fs::remove(path);
}

TEST(vehicle_eta_calibration, drops_batches_beyond_the_pending_byte_limit) {
  auto const dir =
      fs::temp_directory_path() / "motis-eta-calibration-bounded-test";
  fs::remove_all(dir);
  auto store = vehicle_eta_calibration{
      dir, vehicle_eta_calibration::limits{.max_pending_bytes_ = 1U}};
  auto value = candidate(1'600);
  static_cast<void>(store.ingest({&value, 1U}, 1'000, 60));
  value.batch_.observed_passages_ = {{.static_stop_sequence_ = 4U,
                                      .observed_timestamp_seconds_ = 1'640,
                                      .uncertainty_seconds_ = 20}};

  auto const completed = store.ingest({&value, 1U}, 1'650, 60);

  EXPECT_EQ(0U, completed.queued_records_);
  EXPECT_EQ(1U, completed.dropped_records_);
  EXPECT_GT(completed.dropped_bytes_, 1U);
  store.flush();
  EXPECT_FALSE(fs::exists(dir / "calibration.jsonl"));
}

TEST(vehicle_eta_calibration,
     reports_persistence_failures_without_deadlocking) {
  auto const parent =
      fs::temp_directory_path() / "motis-eta-calibration-invalid-parent";
  fs::remove_all(parent);
  {
    auto output = std::ofstream{parent};
    output << "not a directory";
  }
  auto store = vehicle_eta_calibration{parent / "calibration"};
  auto value = candidate(1'600);
  static_cast<void>(store.ingest({&value, 1U}, 1'000, 60));
  value.batch_.observed_passages_ = {{.static_stop_sequence_ = 4U,
                                      .observed_timestamp_seconds_ = 1'640,
                                      .uncertainty_seconds_ = 20}};
  auto const queued = store.ingest({&value, 1U}, 1'650, 60);
  ASSERT_EQ(1U, queued.queued_records_);

  store.flush();
  auto const after_failure = store.ingest({}, 1'660, 60);

  EXPECT_EQ(1U, after_failure.persistence_failed_records_);
  EXPECT_EQ(queued.queued_bytes_, after_failure.persistence_failed_bytes_);
  fs::remove(parent);
}

TEST(vehicle_eta_calibration, destruction_drains_queued_records) {
  auto const dir =
      fs::temp_directory_path() / "motis-eta-calibration-drain-test";
  fs::remove_all(dir);
  {
    auto store = vehicle_eta_calibration{dir};
    auto value = candidate(1'600);
    static_cast<void>(store.ingest({&value, 1U}, 1'000, 60));
    value.batch_.observed_passages_ = {{.static_stop_sequence_ = 4U,
                                        .observed_timestamp_seconds_ = 1'640,
                                        .uncertainty_seconds_ = 20}};
    auto const queued = store.ingest({&value, 1U}, 1'650, 60);
    ASSERT_EQ(1U, queued.queued_records_);
  }

  EXPECT_EQ(
      1U,
      vehicle_eta_calibration::read_records(dir / "calibration.jsonl").size());
  fs::remove_all(dir);
}

}  // namespace
}  // namespace motis
