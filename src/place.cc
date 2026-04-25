#include "motis/place.h"

#include <variant>

#include "utl/verify.h"

#include "osr/location.h"
#include "osr/platforms.h"

#include "nigiri/rt/frun.h"
#include "nigiri/special_stations.h"
#include "nigiri/timetable.h"

#include "motis/parse_location.h"
#include "motis/canonical_stop_registry.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/clasz_to_mode.h"

namespace n = nigiri;

namespace motis {

namespace {

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

std::optional<std::vector<api::ModeEnum>> resolve_modes(
    n::timetable const& tt,
    adr_ext const* ae,
    n::location_idx_t const l) {
  if (ae != nullptr) {
    auto mask = n::routing::clasz_mask_t{0U};
    if (!ae->location_clasz_.empty()) {
      mask = ae->location_clasz_.at(l);
    }
    auto const root = tt.locations_.get_root_idx(l);
    if (mask == 0U && root != n::location_idx_t::invalid() &&
        !ae->location_place_.empty()) {
      auto const place_idx = ae->location_place_.at(root);
      if (place_idx != adr_extra_place_idx_t::invalid() &&
          !ae->place_clasz_.empty()) {
        mask = ae->place_clasz_.at(place_idx);
      }
    }
    if (mask != 0U) {
      return to_modes(mask, 5);
    }
  }

  auto mask = n::routing::clasz_mask_t{0U};
  visit_descendants(tt, l, [&](n::location_idx_t const current) {
    for (auto const route : tt.location_routes_[current]) {
      mask |= n::routing::to_mask(tt.route_clasz_[route]);
    }
  });
  return mask == 0U ? std::optional<std::vector<api::ModeEnum>>{}
                    : std::optional<std::vector<api::ModeEnum>>{
                          to_modes(mask, 5)};
}

}  // namespace

tt_location::tt_location(nigiri::rt::run_stop const& stop)
    : l_{stop.get_location_idx()},
      scheduled_{stop.get_scheduled_location_idx()} {}

tt_location::tt_location(nigiri::location_idx_t const l,
                         nigiri::location_idx_t const scheduled)
    : l_{l},
      scheduled_{scheduled == n::location_idx_t::invalid() ? l : scheduled} {}

api::Place to_place(osr::location const l,
                    std::string_view name,
                    std::optional<std::string> const& tz) {
  return {
      .name_ = std::string{name},
      .lat_ = l.pos_.lat_,
      .lon_ = l.pos_.lng_,
      .level_ = l.lvl_.to_float(),
      .tz_ = tz,
      .vertexType_ = api::VertexTypeEnum::NORMAL,
  };
}

osr::level_t get_lvl(osr::ways const* w,
                     osr::platforms const* pl,
                     platform_matches_t const* matches,
                     n::location_idx_t const l) {
  return w && pl && matches ? pl->get_level(*w, (*matches).at(l))
                            : osr::kNoLevel;
}

double get_level(osr::ways const* w,
                 osr::platforms const* pl,
                 platform_matches_t const* matches,
                 n::location_idx_t const l) {
  return get_lvl(w, pl, matches, l).to_float();
}

osr::location get_location(api::Place const& p) {
  return {{p.lat_, p.lon_}, osr::level_t{static_cast<float>(p.level_)}};
}

osr::location get_location(n::timetable const* tt,
                           osr::ways const* w,
                           osr::platforms const* pl,
                           platform_matches_t const* matches,
                           place_t const loc,
                           place_t const start,
                           place_t const dest) {
  return std::visit(
      utl::overloaded{
          [&](osr::location const& l) { return l; },
          [&](tt_location const l) {
            auto l_idx = l.l_;
            if (l_idx == static_cast<n::location_idx_t::value_t>(
                             n::special_station::kStart)) {
              if (std::holds_alternative<osr::location>(start)) {
                return std::get<osr::location>(start);
              }
              l_idx = std::get<tt_location>(start).l_;
            } else if (l_idx == static_cast<n::location_idx_t::value_t>(
                                    n::special_station::kEnd)) {
              if (std::holds_alternative<osr::location>(dest)) {
                return std::get<osr::location>(dest);
              }
              l_idx = std::get<tt_location>(dest).l_;
            }
            utl::verify(tt != nullptr,
                        "resolving stop coordinates: timetable not set");
            return osr::location{tt->locations_.coordinates_.at(l_idx),
                                 get_lvl(w, pl, matches, l_idx)};
          }},
      loc);
}

api::Place to_place(n::timetable const* tt,
                    tag_lookup const* tags,
                    canonical_stop_registry const* csr,
                    osr::ways const* w,
                    osr::platforms const* pl,
                    platform_matches_t const* matches,
                    adr_ext const* ae,
                    tz_map_t const* tz_map,
                    n::lang_t const& lang,
                    place_t const l,
                    place_t const start,
                    place_t const dest,
                    std::string_view name,
                    std::optional<std::string> const& fallback_tz) {
  return std::visit(
      utl::overloaded{
          [&](osr::location const& l) {
            return to_place(l, name, fallback_tz);
          },
          [&](tt_location const tt_l) -> api::Place {
            utl::verify(tt && tags, "resolving stops requires timetable");

            auto l = tt_l.l_;
            if (l == n::get_special_station(n::special_station::kStart)) {
              if (std::holds_alternative<osr::location>(start)) {
                return to_place(std::get<osr::location>(start), "START",
                                fallback_tz);
              }
              l = std::get<tt_location>(start).l_;
            } else if (l == n::get_special_station(n::special_station::kEnd)) {
              if (std::holds_alternative<osr::location>(dest)) {
                return to_place(std::get<osr::location>(dest), "END",
                                fallback_tz);
              }
              l = std::get<tt_location>(dest).l_;
            }
            auto const get_track = [&](n::location_idx_t const x) {
              auto const p =
                  tt->translate(lang, tt->locations_.platform_codes_.at(x));
              return p.empty() ? std::nullopt : std::optional{std::string{p}};
            };

            // check if description is available, if not, return nullopt
            auto const get_description = [&](n::location_idx_t const x) {
              auto const p =
                  tt->translate(lang, tt->locations_.descriptions_.at(x));
              return p.empty() ? std::nullopt : std::optional{std::string{p}};
            };

            auto const pos = tt->locations_.coordinates_[l];
            auto const p = tt->locations_.get_root_idx(l);
            auto const canonical = csr == nullptr ? nullptr : csr->get(l);
            auto const timezone =
                canonical != nullptr
                    ? canonical->timezone_.transform(
                          [](std::string const& x) { return x; })
                    : get_tz(*tt, ae, tz_map, p) == nullptr
                          ? std::optional<std::string>{}
                          : std::optional<std::string>{
                                get_tz(*tt, ae, tz_map, p)->name()};
            auto const name_root =
                canonical == nullptr ? p : canonical->representative_;
            auto const description_root =
                canonical == nullptr ? tt_l.scheduled_ : canonical->representative_;
            auto const canonical_pos =
                canonical == nullptr ? pos : canonical->coordinates_;
            auto const modes =
                canonical != nullptr ? canonical->modes_ : resolve_modes(*tt, ae, l);

            return {
                .name_ = std::string{tt->translate(
                    lang, tt->locations_.names_.at(name_root))},
                .stopId_ = canonical == nullptr ? tags->id(*tt, l)
                                                : canonical->stop_id_,
                .parentId_ = canonical != nullptr
                                 ? std::nullopt
                                 : p == n::location_idx_t::invalid() || p == l
                                       ? std::nullopt
                                       : std::optional{tags->id(*tt, p)},
                .importance_ =
                    canonical != nullptr
                        ? canonical->importance_
                        : ae == nullptr
                              ? std::optional<double>{}
                              : std::optional<double>{
                                    ae->place_importance_.at(
                                        ae->location_place_.at(l))},
                .lat_ = canonical_pos.lat(),
                .lon_ = canonical_pos.lng(),
                .level_ = get_level(w, pl, matches, l),
                .tz_ = timezone.or_else([&]() { return fallback_tz; }),
                .scheduledTrack_ = get_track(tt_l.scheduled_),
                .track_ = get_track(tt_l.l_),
                .description_ = get_description(description_root)
                                    .or_else([&]() {
                                      return canonical == nullptr
                                                 ? std::optional<std::string>{}
                                                 : get_description(tt_l.scheduled_);
                                    }),
                .vertexType_ = api::VertexTypeEnum::TRANSIT,
                .modes_ = std::move(modes)};
          }},
      l);
}

api::Place to_place(n::timetable const* tt,
                    tag_lookup const* tags,
                    canonical_stop_registry const* csr,
                    osr::ways const* w,
                    osr::platforms const* pl,
                    platform_matches_t const* matches,
                    adr_ext const* ae,
                    tz_map_t const* tz_map,
                    n::lang_t const& lang,
                    n::rt::run_stop const& s,
                    place_t const start,
                    place_t const dest) {
  auto const run_cancelled = s.fr_->is_cancelled();
  auto const fallback_tz = s.get_tz_name(
      s.stop_idx_ == 0 ? n::event_type::kDep : n::event_type::kArr);
  auto p = to_place(tt, tags, csr, w, pl, matches, ae, tz_map, lang,
                    tt_location{s}, start, dest, "", fallback_tz);
  p.pickupType_ = !run_cancelled && s.in_allowed()
                      ? api::PickupDropoffTypeEnum::NORMAL
                      : api::PickupDropoffTypeEnum::NOT_ALLOWED;
  p.dropoffType_ = !run_cancelled && s.out_allowed()
                       ? api::PickupDropoffTypeEnum::NORMAL
                       : api::PickupDropoffTypeEnum::NOT_ALLOWED;
  p.cancelled_ = run_cancelled || (!s.in_allowed() && !s.out_allowed() &&
                                   (s.get_scheduled_stop().in_allowed() ||
                                    s.get_scheduled_stop().out_allowed()));
  return p;
}

place_t get_place(n::timetable const* tt,
                  tag_lookup const* tags,
                  canonical_stop_registry const* csr,
                  std::string_view input) {
  if (auto const location = parse_location(input); location.has_value()) {
    return *location;
  }
  utl::verify(tt != nullptr && tags != nullptr,
              R"(could not parse location (no timetable loaded): "{}")", input);
  auto const l = find_stop_location(*tt, *tags, csr, input);
  utl::verify(l.has_value(), R"(could not parse stop location: "{}")", input);
  return tt_location{*l};
}

}  // namespace motis
