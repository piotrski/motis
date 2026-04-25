#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "geo/latlng.h"

#include "nigiri/types.h"

#include "motis-api/motis-api.h"
#include "motis/adr_extend_tt.h"
#include "motis/config.h"
#include "motis/fwd.h"
#include "motis/types.h"

namespace motis {

struct canonical_stop_registry {
  struct canonical_stop {
    nigiri::location_idx_t root_{nigiri::location_idx_t::invalid()};
    nigiri::location_idx_t representative_{nigiri::location_idx_t::invalid()};
    std::string stop_id_{};
    std::vector<nigiri::location_idx_t> members_{};
    geo::latlng coordinates_{};
    std::optional<std::vector<api::ModeEnum>> modes_{};
    std::optional<double> importance_{};
    std::optional<std::string> timezone_{};
  };

  canonical_stop_registry() = default;
  canonical_stop_registry(config::timetable const&,
                          nigiri::timetable const&,
                          tag_lookup const&,
                          adr_ext const*,
                          tz_map_t const*);

  bool enabled() const { return enabled_; }

  nigiri::location_idx_t canonical_root(nigiri::location_idx_t) const;
  bool is_coalesced(nigiri::location_idx_t) const;

  canonical_stop const* get(nigiri::location_idx_t) const;
  canonical_stop const* get(std::string_view canonical_stop_id) const;
  std::optional<nigiri::location_idx_t> find_location(std::string_view) const;

private:
  static bool is_null_island(geo::latlng const&);

  bool enabled_{false};
  vector_map<nigiri::location_idx_t, nigiri::location_idx_t>
      canonical_root_by_location_{};
  hash_map<nigiri::location_idx_t, canonical_stop> stops_{};
  hash_map<std::string, nigiri::location_idx_t> canonical_id_to_root_{};
  hash_map<std::string, nigiri::location_idx_t> raw_stop_id_to_location_{};
};

std::optional<nigiri::location_idx_t> find_stop_location(
    nigiri::timetable const&,
    tag_lookup const&,
    canonical_stop_registry const*,
    std::string_view);

}  // namespace motis
