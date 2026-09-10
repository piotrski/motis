#include "gtest/gtest.h"

#include <limits>

#include "motis/rt/vehicle_prediction_effective.h"

namespace motis {
namespace {

nigiri::transport transport(unsigned const index) {
  return {nigiri::transport_idx_t{index}, nigiri::day_idx_t{1U}};
}

timing_source_candidate candidate(vehicle_prediction_source const source,
                                  std::int64_t const reference,
                                  double const confidence = 1.0) {
  return {
      .source_ = source,
      .reference_timestamp_seconds_ = reference,
      .confidence_ = confidence,
      .physically_reachable_ = true,
      .predictions_ = {{.static_stop_sequence_ = 2U,
                        .event_type_ = vehicle_prediction_event_type::kArrival,
                        .scheduled_timestamp_seconds_ = 1'000,
                        .predicted_timestamp_seconds_ = 1'125,
                        .delay_seconds_ = 125}}};
}

TEST(vehicle_prediction_effective, prefers_eligible_gps_and_allows_gps_only) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5};
  auto input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'050, 0.5)};
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.provider_.reset();
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kGps);
}

TEST(vehicle_prediction_effective,
     supports_a_provider_without_a_gps_candidate) {
  auto const policy = effective_vehicle_prediction_policy{};
  auto const input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080)};

  auto const selection = select_effective_vehicle_prediction(input, policy);

  EXPECT_EQ(selection.source_, vehicle_prediction_source::kProvider);
  EXPECT_FALSE(selection.diagnostics_.gps_rejection_.has_value());
}

TEST(vehicle_prediction_effective, provider_wins_when_gps_is_too_old) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5};
  auto input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'019, 0.9)};
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kProvider);
  input.gps_->reference_timestamp_seconds_ = 1'020;
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kGps);
}

TEST(vehicle_prediction_effective,
     future_dated_provider_does_not_suppress_fresh_gps) {
  auto const policy = effective_vehicle_prediction_policy{};
  auto input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 10'000),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'090, 0.9)};

  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.gps_.reset();
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kSchedule);
}

TEST(vehicle_prediction_effective, rejects_stale_and_low_confidence_gps) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5};
  auto input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'500,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'490),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'199, 0.9)};
  auto selection = select_effective_vehicle_prediction(input, policy);
  EXPECT_EQ(selection.source_, vehicle_prediction_source::kProvider);
  EXPECT_EQ(selection.diagnostics_.gps_rejection_,
            timing_candidate_rejection_reason::kStale);
  input.gps_->reference_timestamp_seconds_ = 1'450;
  input.gps_->confidence_ = 0.499;
  selection = select_effective_vehicle_prediction(input, policy);
  EXPECT_EQ(selection.source_, vehicle_prediction_source::kProvider);
  EXPECT_EQ(selection.diagnostics_.gps_rejection_,
            timing_candidate_rejection_reason::kLowConfidence);
}

TEST(vehicle_prediction_effective_state,
     gps_must_enter_at_high_threshold_but_can_remain_at_low_threshold) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5,
      .min_selected_gps_confidence_ = 0.35};
  auto state = effective_vehicle_prediction_selection_state{};
  auto input = vehicle_prediction_selection_input{
      .transport_ = transport(1U),
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'050, 0.49)};

  EXPECT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kProvider);
  input.now_seconds_ += 10;
  input.gps_->reference_timestamp_seconds_ += 10;
  input.gps_->confidence_ = 0.5;
  EXPECT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.now_seconds_ += 10;
  input.gps_->reference_timestamp_seconds_ += 10;
  input.gps_->confidence_ = 0.35;
  auto const selected = state.select(input, policy);
  EXPECT_EQ(selected.source_, vehicle_prediction_source::kGps);
  EXPECT_EQ(selected.reason_,
            vehicle_prediction_selection_reason::kSourceHysteresis);

  input.now_seconds_ += 10;
  input.gps_->reference_timestamp_seconds_ += 10;
  input.gps_->confidence_ = 0.349;
  EXPECT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kProvider);

  input.transport_ = transport(2U);
  input.gps_->confidence_ = 0.4;
  EXPECT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kProvider);
}

TEST(vehicle_prediction_effective_state,
     hard_gps_failures_bypass_source_hysteresis) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5,
      .min_selected_gps_confidence_ = 0.35};
  auto state = effective_vehicle_prediction_selection_state{};
  auto input = vehicle_prediction_selection_input{
      .transport_ = transport(1U),
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'050, 0.8)};
  ASSERT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);

  input.now_seconds_ = 1'400;
  input.gps_->reference_timestamp_seconds_ = 1'099;
  auto selected = state.select(input, policy);
  EXPECT_EQ(selected.source_, vehicle_prediction_source::kProvider);
  EXPECT_EQ(selected.diagnostics_.gps_rejection_,
            timing_candidate_rejection_reason::kStale);

  input.now_seconds_ += 1;
  input.gps_->reference_timestamp_seconds_ = input.now_seconds_;
  ASSERT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.now_seconds_ += 1;
  input.gps_->reference_timestamp_seconds_ = input.now_seconds_;
  input.gps_->physically_reachable_ = false;
  selected = state.select(input, policy);
  EXPECT_EQ(selected.source_, vehicle_prediction_source::kProvider);
  EXPECT_EQ(selected.diagnostics_.gps_rejection_,
            timing_candidate_rejection_reason::kPhysicallyUnreachable);

  input.gps_->physically_reachable_ = true;
  ASSERT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.now_seconds_ += 1;
  input.gps_->reference_timestamp_seconds_ = input.now_seconds_;
  input.gps_->predictions_.clear();
  selected = state.select(input, policy);
  EXPECT_EQ(selected.source_, vehicle_prediction_source::kProvider);
  EXPECT_EQ(selected.diagnostics_.gps_rejection_,
            timing_candidate_rejection_reason::kPhysicallyUnreachable);
}

TEST(vehicle_prediction_effective_state,
     expires_old_entries_and_can_clear_runtime_disabled_trips) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 60,
      .min_gps_confidence_ = 0.5,
      .min_selected_gps_confidence_ = 0.35,
      .selection_state_ttl_seconds_ = 120};
  auto state = effective_vehicle_prediction_selection_state{};
  auto input = vehicle_prediction_selection_input{
      .transport_ = transport(1U),
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'050, 0.8)};
  ASSERT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);

  state.erase(input.transport_, {});
  input.now_seconds_ += 10;
  input.gps_->reference_timestamp_seconds_ += 10;
  input.gps_->confidence_ = 0.4;
  EXPECT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kProvider);

  input.transport_ = transport(2U);
  input.gps_->confidence_ = 0.8;
  ASSERT_EQ(state.select(input, policy).source_,
            vehicle_prediction_source::kGps);
  state.expire(input.now_seconds_ + 121, policy);
  EXPECT_EQ(state.size(), 0U);
}

TEST(vehicle_prediction_effective_state,
     scopes_hysteresis_and_erasure_to_an_interlined_trip_range) {
  auto const policy = effective_vehicle_prediction_policy{
      .min_gps_confidence_ = 0.5,
      .min_selected_gps_confidence_ = 0.35};
  auto state = effective_vehicle_prediction_selection_state{};
  auto input = vehicle_prediction_selection_input{
      .transport_ = transport(1U),
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 1'090, 0.8)};
  auto const first_range = nigiri::interval<nigiri::stop_idx_t>{
      nigiri::stop_idx_t{0U}, nigiri::stop_idx_t{2U}};
  auto const next_range = nigiri::interval<nigiri::stop_idx_t>{
      nigiri::stop_idx_t{2U}, nigiri::stop_idx_t{4U}};
  ASSERT_EQ(state.select(input, policy, first_range).source_,
            vehicle_prediction_source::kGps);

  auto next = input;
  next.gps_->confidence_ = 0.4;
  EXPECT_EQ(state.select(next, policy, next_range).source_,
            vehicle_prediction_source::kProvider);
  state.erase(next.transport_, next_range);

  input.now_seconds_ += 1;
  input.gps_->confidence_ = 0.4;
  EXPECT_EQ(state.select(input, policy, first_range).source_,
            vehicle_prediction_source::kGps);
}

TEST(vehicle_prediction_effective,
     provider_comparison_uses_explicit_timestamp_tolerance) {
  auto const policy = effective_vehicle_prediction_policy{
      .max_gps_age_seconds_ = 300,
      .provider_timestamp_tolerance_seconds_ = 90,
      .min_gps_confidence_ = 0.5,
      .min_selected_gps_confidence_ = 0.35};
  auto input = vehicle_prediction_selection_input{
      .now_seconds_ = 1'100,
      .provider_ = candidate(vehicle_prediction_source::kProvider, 1'080),
      .gps_ = candidate(vehicle_prediction_source::kGps, 990, 0.9)};

  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kGps);
  input.gps_->reference_timestamp_seconds_ = 989;
  EXPECT_EQ(select_effective_vehicle_prediction(input, policy).source_,
            vehicle_prediction_source::kProvider);
}

TEST(vehicle_prediction_effective, rounds_halves_away_and_keeps_deadband) {
  EXPECT_EQ(round_delay_minutes(29, std::nullopt, 10), 0);
  EXPECT_EQ(round_delay_minutes(30, std::nullopt, 10), 1);
  EXPECT_EQ(round_delay_minutes(-30, std::nullopt, 10), -1);
  EXPECT_EQ(round_delay_minutes(35, 0, 10), 0);
  EXPECT_EQ(round_delay_minutes(40, 0, 10), 1);
  EXPECT_EQ(round_delay_minutes(-35, 0, 10), 0);
  EXPECT_EQ(round_delay_minutes(-40, 0, 10), -1);
}

TEST(vehicle_prediction_effective,
     rounds_extreme_delays_without_overflow) {
  EXPECT_EQ(round_delay_minutes(std::numeric_limits<std::int64_t>::max(),
                                std::nullopt, 10),
            std::numeric_limits<std::int64_t>::max() / 60);
  EXPECT_EQ(round_delay_minutes(std::numeric_limits<std::int64_t>::min(),
                                std::nullopt, 10),
            std::numeric_limits<std::int64_t>::min() / 60);
}

}  // namespace
}  // namespace motis
