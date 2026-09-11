#include "gtest/gtest.h"

#include "motis/rt/vehicle_prediction_store.h"

namespace motis {
namespace {

auto entry(unsigned const transport, unsigned const stop, std::int64_t seen) {
  return vehicle_prediction_diagnostic_entry{
      .transport_ = nigiri::transport{nigiri::transport_idx_t{transport},
                                      nigiri::day_idx_t{0U}},
      .static_stop_sequence_ = stop,
      .trip_id_ = "feed_trip",
      .observed_at_seconds_ = seen,
      .scheduled_timestamp_seconds_ =
          static_cast<std::int64_t>(stop) * 100 - 10,
      .effective_ = {
          .predicted_timestamp_seconds_ = static_cast<std::int64_t>(stop) * 100,
          .delay_seconds_ = 10}};
}

TEST(vehicle_prediction_store, allocates_nothing_when_disabled) {
  EXPECT_EQ(nullptr, vehicle_prediction_diagnostics_store::build(
                         false, {entry(1U, 2U, 100)}, 100));
}

TEST(vehicle_prediction_store, unchanged_realtime_event_remains_scheduled) {
  auto const unchanged = resolve_effective_prediction(true, 100, 100, nullptr);
  EXPECT_EQ(vehicle_prediction_source::kSchedule, unchanged.source_);

  auto const delayed = resolve_effective_prediction(true, 100, 110, nullptr);
  EXPECT_EQ(vehicle_prediction_source::kProvider, delayed.source_);
}

TEST(vehicle_prediction_store, bounds_expires_deduplicates_and_finds) {
  auto store = vehicle_prediction_diagnostics_store::build(
      true,
      {entry(2U, 4U, 95), entry(1U, 3U, 99), entry(1U, 3U, 98),
       entry(3U, 5U, 1)},
      100, {.max_age_seconds_ = 10, .max_entries_ = 2U});
  ASSERT_NE(nullptr, store);
  ASSERT_EQ(2U, store->size());
  ASSERT_NE(nullptr, store->find(nigiri::transport{nigiri::transport_idx_t{1U},
                                                   nigiri::day_idx_t{0U}},
                                 3U));
  EXPECT_EQ(99, store
                    ->find(nigiri::transport{nigiri::transport_idx_t{1U},
                                             nigiri::day_idx_t{0U}},
                           3U)
                    ->observed_at_seconds_);
  EXPECT_EQ(nullptr, store->find(nigiri::transport{nigiri::transport_idx_t{3U},
                                                   nigiri::day_idx_t{0U}},
                                 5U));
  EXPECT_NE(
      nullptr,
      store->find_event(
          nigiri::transport{nigiri::transport_idx_t{1U}, nigiri::day_idx_t{0U}},
          "feed_trip", 3U, 290, vehicle_prediction_event_type::kArrival));
}

TEST(vehicle_prediction_store, event_lookup_distinguishes_repeated_times) {
  auto first = entry(1U, 2U, 100);
  auto second = entry(1U, 3U, 100);
  second.scheduled_timestamp_seconds_ = first.scheduled_timestamp_seconds_;
  second.effective_.predicted_timestamp_seconds_ = 999;
  auto store = vehicle_prediction_diagnostics_store::build(
      true, {std::move(first), std::move(second)}, 100);

  ASSERT_NE(nullptr, store);
  auto const* found = store->find_event(
      nigiri::transport{nigiri::transport_idx_t{1U}, nigiri::day_idx_t{0U}},
      "feed_trip", 3U, 190, vehicle_prediction_event_type::kArrival);
  ASSERT_NE(nullptr, found);
  EXPECT_EQ(found->effective_.predicted_timestamp_seconds_, 999);
}

TEST(vehicle_prediction_store, bounds_preserve_effective_gps_predictions) {
  auto gps = entry(9U, 1U, 100);
  gps.effective_.source_ = vehicle_prediction_source::kGps;
  auto store = vehicle_prediction_diagnostics_store::build(
      true, {entry(1U, 1U, 100), entry(2U, 1U, 100), std::move(gps)}, 100,
      {.max_entries_ = 2U});

  ASSERT_NE(nullptr, store);
  ASSERT_EQ(2U, store->size());
  EXPECT_NE(nullptr, store->find(nigiri::transport{nigiri::transport_idx_t{9U},
                                                   nigiri::day_idx_t{0U}},
                                 1U));
}

}  // namespace
}  // namespace motis
