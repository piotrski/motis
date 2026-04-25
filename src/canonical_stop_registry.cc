#include "motis/canonical_stop_registry.h"

#include <algorithm>
#include <numeric>
#include <vector>

#include "utl/erase_duplicates.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"

#include "nigiri/routing/clasz_mask.h"
#include "nigiri/special_stations.h"
#include "nigiri/timetable.h"

#include "motis/adr_extend_tt.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/clasz_to_mode.h"

namespace n = nigiri;

namespace motis {

namespace {

struct disjoint_set {
  explicit disjoint_set(std::size_t const size) : parent_(size), rank_(size, 0U) {
    std::iota(begin(parent_), end(parent_), 0U);
  }

  std::size_t find(std::size_t x) {
    if (parent_[x] != x) {
      parent_[x] = find(parent_[x]);
    }
    return parent_[x];
  }

  void unite(std::size_t const a, std::size_t const b) {
    auto root_a = find(a);
    auto root_b = find(b);
    if (root_a == root_b) {
      return;
    }
    if (rank_[root_a] < rank_[root_b]) {
      std::swap(root_a, root_b);
    }
    parent_[root_b] = root_a;
    if (rank_[root_a] == rank_[root_b]) {
      ++rank_[root_a];
    }
  }

  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
};

template <typename Fn>
void visit_descendants(n::timetable const& tt,
                       n::location_idx_t const root,
                       Fn&& fn) {
  auto queue = std::vector<n::location_idx_t>{root};
  for (auto i = std::size_t{0U}; i != queue.size(); ++i) {
    auto const current = queue[i];
    fn(current);
    for (auto const child : tt.locations_.children_[current]) {
      queue.emplace_back(child);
    }
  }
}

}  // namespace

bool canonical_stop_registry::is_null_island(geo::latlng const& pos) {
  return pos.lat() < 3.0 && pos.lng() < 3.0;
}

canonical_stop_registry::canonical_stop_registry(config::timetable const& config,
                                                 n::timetable const& tt,
                                                 tag_lookup const& tags,
                                                 adr_ext const* ae,
                                                 tz_map_t const* tz_map) {
  if (!config.coalesce_equivalent_stops_) {
    return;
  }

  enabled_ = true;
  canonical_root_by_location_.resize(tt.n_locations(),
                                     n::location_idx_t::invalid());

  auto roots = std::vector<n::location_idx_t>{};
  auto root_to_idx = hash_map<n::location_idx_t, std::size_t>{};
  auto default_name = hash_map<n::location_idx_t, std::string_view>{};

  for (auto i = n::kNSpecialStations; i < tt.n_locations(); ++i) {
    auto const l = n::location_idx_t{i};
    raw_stop_id_to_location_.emplace(tags.id(tt, l), l);

    if (tt.locations_.get_root_idx(l) != l ||
        tt.locations_.src_[l] == n::source_idx_t::invalid()) {
      continue;
    }

    root_to_idx.emplace(l, roots.size());
    roots.emplace_back(l);
    default_name.emplace(l, tt.get_default_translation(tt.locations_.names_[l]));
  }

  auto dsu = disjoint_set{roots.size()};
  for (auto i = n::kNSpecialStations; i < tt.n_locations(); ++i) {
    auto const l = n::location_idx_t{i};
    auto const root = tt.locations_.get_root_idx(l);
    auto const root_it = root_to_idx.find(root);
    if (root_it == end(root_to_idx)) {
      continue;
    }

    for (auto const eq : tt.locations_.equivalences_[l]) {
      auto const eq_root = tt.locations_.get_root_idx(eq);
      auto const eq_it = root_to_idx.find(eq_root);
      if (eq_it == end(root_to_idx) || root == eq_root ||
          default_name[root] != default_name[eq_root]) {
        continue;
      }
      dsu.unite(root_it->second, eq_it->second);
    }
  }

  auto groups = hash_map<std::size_t, std::vector<n::location_idx_t>>{};
  for (auto const [idx, root] : utl::enumerate(roots)) {
    groups[dsu.find(idx)].emplace_back(root);
  }

  for (auto& [_, members] : groups) {
    utl::sort(members, [&](auto const a, auto const b) {
      return tags.id(tt, a) < tags.id(tt, b);
    });
    if (members.size() <= 1U) {
      continue;
    }

    auto const canonical_root = members.front();
    auto stop = canonical_stop{
        .root_ = canonical_root,
        .representative_ = canonical_root,
        .stop_id_ = tags.id(tt, canonical_root),
        .members_ = members,
    };

    auto const rep_place_idx =
        ae == nullptr ? adr_extra_place_idx_t::invalid()
                      : ae->location_place_[canonical_root];
    if (rep_place_idx != adr_extra_place_idx_t::invalid()) {
      stop.importance_ = ae->place_importance_[rep_place_idx];
    }

    auto mode_mask = n::routing::clasz_mask_t{0U};
    auto importance = stop.importance_;
    auto lat_sum = 0.0;
    auto lon_sum = 0.0;
    auto coord_count = std::size_t{0U};
    for (auto const member_root : members) {
      if (ae != nullptr) {
        auto const place_idx = ae->location_place_[member_root];
        if (place_idx != adr_extra_place_idx_t::invalid()) {
          importance = std::max(
              importance.value_or(0.0),
              static_cast<double>(ae->place_importance_[place_idx]));
        }
      }

      auto const pos = tt.locations_.coordinates_[member_root];
      if (!is_null_island(pos)) {
        lat_sum += pos.lat();
        lon_sum += pos.lng();
        ++coord_count;
      }

      visit_descendants(tt, member_root, [&](n::location_idx_t const l) {
        for (auto const route : tt.location_routes_[l]) {
          mode_mask |= n::routing::to_mask(tt.route_clasz_[route]);
        }
      });
    }

    stop.importance_ = importance;
    stop.coordinates_ =
        coord_count == 0U
            ? tt.locations_.coordinates_[canonical_root]
            : geo::latlng{lat_sum / static_cast<double>(coord_count),
                          lon_sum / static_cast<double>(coord_count)};
    if (mode_mask != 0U) {
      stop.modes_ = to_modes(mode_mask, 5);
    }
    if (auto const* tz = get_tz(tt, ae, tz_map, canonical_root); tz != nullptr) {
      stop.timezone_ = std::string{tz->name()};
    }

    for (auto const member_root : members) {
      visit_descendants(tt, member_root, [&](n::location_idx_t const l) {
        canonical_root_by_location_[l] = canonical_root;
      });
    }

    canonical_id_to_root_.emplace(stop.stop_id_, canonical_root);
    stops_.emplace(canonical_root, std::move(stop));
  }
}

n::location_idx_t canonical_stop_registry::canonical_root(
    n::location_idx_t const l) const {
  if (!enabled_ || l == n::location_idx_t::invalid() ||
      to_idx(l) >= canonical_root_by_location_.size()) {
    return n::location_idx_t::invalid();
  }
  return canonical_root_by_location_[l];
}

bool canonical_stop_registry::is_coalesced(n::location_idx_t const l) const {
  return canonical_root(l) != n::location_idx_t::invalid();
}

canonical_stop_registry::canonical_stop const* canonical_stop_registry::get(
    n::location_idx_t const l) const {
  auto const root = canonical_root(l);
  if (root == n::location_idx_t::invalid()) {
    return nullptr;
  }
  if (auto const it = stops_.find(root); it != end(stops_)) {
    return &it->second;
  }
  return nullptr;
}

canonical_stop_registry::canonical_stop const* canonical_stop_registry::get(
    std::string_view const canonical_stop_id) const {
  if (auto const l = find_location(canonical_stop_id); l.has_value()) {
    return get(*l);
  }
  return nullptr;
}

std::optional<n::location_idx_t> canonical_stop_registry::find_location(
    std::string_view const id) const {
  if (!enabled_) {
    return std::nullopt;
  }
  if (auto const canonical_it = canonical_id_to_root_.find(std::string{id});
      canonical_it != end(canonical_id_to_root_)) {
    return canonical_it->second;
  }
  if (auto const raw_it = raw_stop_id_to_location_.find(std::string{id});
      raw_it != end(raw_stop_id_to_location_)) {
    return raw_it->second;
  }
  return std::nullopt;
}

std::optional<n::location_idx_t> find_stop_location(n::timetable const& tt,
                                                    tag_lookup const& tags,
                                                    canonical_stop_registry const* csr,
                                                    std::string_view const id) {
  if (csr != nullptr) {
    if (auto const l = csr->find_location(id); l.has_value()) {
      return l;
    }
  }
  return tags.find_location(tt, id);
}

}  // namespace motis
