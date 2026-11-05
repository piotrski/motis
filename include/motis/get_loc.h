#pragma once

#include "osr/platforms.h"
#include "osr/routing/route.h"

#include "nigiri/timetable.h"

#include "motis/constants.h"
#include "motis/match_platforms.h"
#include "motis/types.h"

namespace motis {

// Position of a stop snapped to the center of its matched platform if that is
// closer than `kMaxAdjust`. This is the position `get_loc` queries OSR with
// and the one `compute_footpaths` moves the stop to for
// `use_osm_stop_coordinates_`.
inline geo::latlng get_adjusted_pos(
    nigiri::timetable const& tt,
    osr::ways const& w,
    osr::platforms const& pl,
    vector_map<nigiri::location_idx_t, osr::platform_idx_t> const& matches,
    nigiri::location_idx_t const l) {
  auto pos = tt.locations_.coordinates_[l];
  if (matches[l] != osr::platform_idx_t::invalid()) {
    auto const center = get_platform_center(pl, w, matches[l]);
    if (center.has_value() && geo::distance(*center, pos) < kMaxAdjust) {
      pos = *center;
    }
  }
  return pos;
}

inline osr::location get_loc(
    nigiri::timetable const& tt,
    osr::ways const& w,
    osr::platforms const& pl,
    vector_map<nigiri::location_idx_t, osr::platform_idx_t> const& matches,
    nigiri::location_idx_t const l) {
  auto const pos = get_adjusted_pos(tt, w, pl, matches, l);
  auto const lvl = matches[l] == osr::platform_idx_t::invalid()
                       ? osr::level_t{0.F}
                       : pl.get_level(w, matches[l]);
  return {pos, lvl};
}

}  // namespace motis