#pragma once

#include "osr/routing/profile.h"

#include "nigiri/routing/query.h"
#include "nigiri/types.h"

namespace motis {

// nigiri treats the transport mode of an offset as opaque. MOTIS keeps its own
// id encoding (osr profile, ODM, ride sharing, GBFS, flex) in the payload and
// leaves transport_mode_t::mode_ at 0.
using transport_mode_id_t = nigiri::routing::transport_mode_t::payload_t;

constexpr auto const kOdmTransportModeId =
    static_cast<transport_mode_id_t>(osr::kNumProfiles);
constexpr auto const kRideSharingTransportModeId =
    static_cast<transport_mode_id_t>(osr::kNumProfiles + 1U);
constexpr auto const kGbfsTransportModeIdOffset =
    static_cast<transport_mode_id_t>(osr::kNumProfiles + 2U);
constexpr auto const kFlexModeIdOffset =
    static_cast<transport_mode_id_t>(1'000'000U);

}  // namespace motis
