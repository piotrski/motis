#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "motis/rt/vehicle_prediction_selection.h"

namespace motis {

struct effective_vehicle_prediction_policy {
  std::int64_t max_gps_age_seconds_{300};
  std::int64_t provider_timestamp_tolerance_seconds_{60};
  double min_gps_confidence_{0.5};
  double min_selected_gps_confidence_{0.35};
  std::int64_t selection_state_ttl_seconds_{300};
};

// Production policy: eligible GPS wins unless its observation is older than
// the provider candidate by more than the explicit timestamp tolerance.
[[nodiscard]] vehicle_prediction_selection select_effective_vehicle_prediction(
    vehicle_prediction_selection_input const&,
    effective_vehicle_prediction_policy const&);

// Persists only the selected source for a trip instance. GPS must enter at the
// normal confidence threshold, but a previously selected GPS candidate may
// remain selected at the lower threshold. Candidate validity and timestamp
// checks always take precedence over source persistence.
struct effective_vehicle_prediction_selection_state {
  [[nodiscard]] vehicle_prediction_selection select(
      vehicle_prediction_selection_input const&,
      effective_vehicle_prediction_policy const&,
      std::optional<nigiri::interval<nigiri::stop_idx_t>> const& stop_range =
          std::nullopt);
  void expire(std::int64_t now_seconds,
              effective_vehicle_prediction_policy const&);
  void erase(nigiri::transport const&,
             std::optional<nigiri::interval<nigiri::stop_idx_t>> const&);
  void reset() { entries_.clear(); }
  [[nodiscard]] std::size_t size() const { return entries_.size(); }

private:
  struct entry {
    nigiri::transport transport_{nigiri::transport::invalid()};
    std::optional<nigiri::interval<nigiri::stop_idx_t>> stop_range_{};
    vehicle_prediction_source selected_{vehicle_prediction_source::kSchedule};
    std::int64_t last_seen_seconds_{};
  };

  std::vector<entry> entries_;
};

// Rounds to the nearest minute, halves away from zero. If the rounded value is
// adjacent to the prior value, retain the prior minute inside the deadband.
[[nodiscard]] std::int64_t round_delay_minutes(
    std::int64_t delay_seconds,
    std::optional<std::int64_t> previous_minutes,
    std::int64_t deadband_seconds = 10);

}  // namespace motis
