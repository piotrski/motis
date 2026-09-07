#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "nigiri/common/interval.h"
#include "nigiri/types.h"

#include "motis/rt/vehicle_prediction.h"

namespace nigiri {
struct rt_timetable;
struct timetable;
}  // namespace nigiri

namespace motis {

struct selected_vehicle_prediction_trip {
  nigiri::transport transport_{nigiri::transport::invalid()};
  nigiri::source_idx_t source_{nigiri::source_idx_t::invalid()};
  std::optional<nigiri::interval<nigiri::stop_idx_t>> trip_stop_range_{};
  std::vector<vehicle_stop_prediction> predictions_{};
};

struct vehicle_prediction_overlay_size {
  nigiri::transport transport_{nigiri::transport::invalid()};
  std::size_t event_count_{};
};

[[nodiscard]] std::vector<nigiri::transport>
select_complete_vehicle_prediction_overlays(
    std::span<vehicle_prediction_overlay_size const>,
    std::size_t max_events);

struct vehicle_prediction_overlay_policy {
  std::int64_t early_departure_tolerance_seconds_{0};
  std::int64_t minute_rounding_deadband_seconds_{10};
};

struct vehicle_prediction_overlay_state {
  struct rendered_event {
    nigiri::transport transport_{nigiri::transport::invalid()};
    std::optional<nigiri::interval<nigiri::stop_idx_t>> trip_stop_range_{};
    unsigned static_stop_sequence_{};
    vehicle_prediction_event_type event_type_{
        vehicle_prediction_event_type::kArrival};
    std::int64_t delay_minutes_{};
    std::optional<std::int64_t> effective_timestamp_seconds_{};
    std::optional<std::int64_t> selected_timestamp_seconds_{};
  };

  std::vector<rendered_event> rendered_events_{};
};

struct vehicle_prediction_overlay_result {
  std::size_t applied_trips_{};
  std::size_t rejected_trips_{};
  std::vector<nigiri::transport> applied_transports_{};
};

// Applies each trip transactionally: all event updates are validated before
// the first timetable mutation. Existing cancellation, skip and platform state
// is left untouched.
[[nodiscard]] vehicle_prediction_overlay_result
apply_vehicle_prediction_overlay(
    nigiri::timetable const&,
    nigiri::rt_timetable&,
    std::span<selected_vehicle_prediction_trip const>,
    vehicle_prediction_overlay_policy const&,
    vehicle_prediction_overlay_state&);

}  // namespace motis
