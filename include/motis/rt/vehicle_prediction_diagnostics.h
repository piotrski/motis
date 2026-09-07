#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "nigiri/common/interval.h"
#include "nigiri/types.h"

#include "motis/rt/vehicle_prediction.h"
#include "motis/rt/vehicle_prediction_continuation.h"

namespace nigiri {
struct shapes_storage;
struct timetable;
struct rt_timetable;
}  // namespace nigiri

namespace motis {

struct config;
struct tag_lookup;
struct vehicle_observation_history;
class vehicle_eta_runtime_control;

namespace vehicle_positions {
struct vehicle_position_store;
}

struct vehicle_prediction_cycle_result {
  std::string feed_{};
  std::string trip_id_{};
  std::optional<nigiri::clasz> mode_{};
  std::optional<nigiri::interval<nigiri::stop_idx_t>> trip_stop_range_{};
  vehicle_prediction_context context_{vehicle_prediction_context::kDirect};
  std::optional<incoming_leg_prediction_provenance> incoming_leg_provenance_{};
  std::optional<std::int64_t> latest_vehicle_observation_timestamp_seconds_{};
  vehicle_prediction_batch batch_{};
  std::vector<vehicle_stop_prediction> provider_predictions_{};
  std::optional<std::int64_t> provider_reference_timestamp_seconds_{};
  std::vector<std::int64_t> provider_raw_error_seconds_{};
  std::vector<std::int64_t> provider_minute_error_{};
};

// Runs candidate generation after provider application, against the private
// next snapshot. It is intentionally read-only and never rewrites FeedMessage
// or timetable timing.
[[nodiscard]] std::vector<vehicle_prediction_cycle_result>
evaluate_vehicle_prediction_candidates(
    config const&,
    tag_lookup const&,
    nigiri::timetable const&,
    nigiri::rt_timetable const*,
    nigiri::shapes_storage const*,
    vehicle_positions::vehicle_position_store const&,
    vehicle_observation_history const&,
    std::int64_t now_seconds,
    vehicle_eta_runtime_control const* = nullptr);

}  // namespace motis
