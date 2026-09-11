#pragma once

#include <cstddef>

namespace motis {

inline constexpr std::size_t kMaxVehiclePredictionPositionsPerCycle = 10'000U;
inline constexpr std::size_t kMaxVehiclePredictionResolutionsPerCycle =
    20'000U;
inline constexpr auto kMaxVehiclePredictionDiagnosticEntries =
    std::size_t{100'000U};

}  // namespace motis
