#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "nigiri/common/interval.h"
#include "nigiri/types.h"

#include "motis/rt/vehicle_prediction.h"

namespace nigiri {
struct timetable;
namespace rt {
struct frun;
}
}  // namespace nigiri

namespace motis {

struct tag_lookup;
struct vehicle_prediction_cycle_result;
struct vehicle_prediction_diagnostics_store;

enum class vehicle_prediction_context { kDirect, kIncomingBlockLeg };

struct incoming_leg_prediction_provenance {
  bool operator==(incoming_leg_prediction_provenance const&) const = default;

  std::string feed_;
  std::string incoming_trip_id_;
  std::string next_trip_id_;
  std::string route_id_;
  std::string route_short_name_;
  std::string headsign_;
  nigiri::clasz mode_{nigiri::clasz::kOther};
  std::int64_t expected_terminal_arrival_seconds_{};
  std::int64_t next_scheduled_departure_seconds_{};
  std::int64_t propagated_delay_seconds_{};
  std::int64_t reference_timestamp_seconds_{};
};

struct vehicle_prediction_continuation {
  nigiri::transport transport_{nigiri::transport::invalid()};
  nigiri::interval<nigiri::stop_idx_t> trip_stop_range_{};
  std::string trip_id_;
  nigiri::clasz mode_{nigiri::clasz::kOther};
  std::int64_t reference_timestamp_seconds_{};
  double confidence_{};
  std::vector<vehicle_stop_prediction> predictions_;
  incoming_leg_prediction_provenance provenance_;
};

// Propagates only a verified immediately-adjacent trip on the same scheduled
// block transport. Previous-leg coordinates never cross this boundary.
[[nodiscard]] std::optional<vehicle_prediction_continuation>
derive_vehicle_prediction_continuation(nigiri::timetable const&,
                                       tag_lookup const&,
                                       nigiri::rt::frun const& current_run,
                                       std::string_view feed,
                                       std::string_view current_trip_id,
                                       vehicle_prediction_batch const&);

// Keeps fresh incoming-leg predictions through the transition until direct
// next-leg GPS is eligible. The returned candidates are ready for normal
// provider/GPS selection.
[[nodiscard]] std::vector<vehicle_prediction_cycle_result>
merge_retained_vehicle_prediction_continuations(
    vehicle_prediction_diagnostics_store const* previous,
    std::vector<vehicle_prediction_cycle_result> current,
    std::int64_t now_seconds,
    std::int64_t max_age_seconds);

}  // namespace motis
