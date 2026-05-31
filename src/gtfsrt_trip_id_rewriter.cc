#include "motis/gtfsrt_trip_id_rewriter.h"

#include <string>
#include <string_view>
#include <unordered_set>

#include "fmt/format.h"

#include "utl/enumerate.h"

#include "nigiri/rt/frun.h"

namespace n = nigiri;

namespace motis {

namespace {

std::string normalize_start_time(std::string_view start_time) {
  if (start_time.size() >= 5U && start_time[2] == ':') {
    return std::string{start_time.substr(0U, 5U)};
  }
  return std::string{start_time};
}

std::string make_lookup_key(std::string_view route_id,
                            std::string_view start_date,
                            std::string_view start_time) {
  return fmt::format("{}|{}|{}", route_id, start_date,
                     normalize_start_time(start_time));
}

void add_lookup_entry(gtfsrt_trip_id_lookup_t& lookup,
                      std::unordered_set<std::string>& ambiguous,
                      std::string const& key,
                      std::string const& trip_id) {
  if (ambiguous.contains(key)) {
    return;
  }

  if (auto const it = lookup.find(key); it == end(lookup)) {
    lookup.emplace(key, trip_id);
  } else if (it->second != trip_id) {
    lookup.erase(it);
    ambiguous.emplace(key);
  }
}

bool rewrite_trip_descriptor(transit_realtime::TripDescriptor* td,
                             gtfsrt_trip_id_lookup_t const& lookup) {
  if (td == nullptr || !td->has_trip_id() || !td->has_route_id() ||
      !td->has_start_date() || !td->has_start_time() ||
      td->trip_id().find(':') == std::string::npos) {
    return false;
  }

  auto const key =
      make_lookup_key(td->route_id(), td->start_date(), td->start_time());
  auto const it = lookup.find(key);
  if (it == end(lookup) || it->second == td->trip_id()) {
    return false;
  }

  td->set_trip_id(it->second);
  return true;
}

}  // namespace

gtfsrt_trip_id_lookup_t build_gtfsrt_trip_id_lookup(
    n::timetable const& tt,
    tag_lookup const& tags,
    n::rt_timetable const& rtt,
    std::string_view const tag) {
  auto lookup = gtfsrt_trip_id_lookup_t{};
  auto ambiguous = std::unordered_set<std::string>{};
  auto const day_idx_iv =
      n::interval{tt.day_idx(date::floor<date::days>(tt.internal_interval().from_)),
                  tt.day_idx(date::floor<date::days>(tt.internal_interval().to_))};

  for (auto r = n::route_idx_t{0}; r < tt.n_routes(); ++r) {
    for (auto const t_idx : tt.route_transport_ranges_[r]) {
      auto const& bitfield = tt.bitfields_[tt.transport_traffic_days_[t_idx]];
      for (auto const day_idx : day_idx_iv) {
        if (!bitfield.test(cista::to_idx(day_idx))) {
          continue;
        }

        auto const fr = n::rt::frun::from_t(tt, &rtt,
                                            n::transport{t_idx, day_idx});
        fr.for_each_trip([&](n::trip_idx_t,
                             n::interval<n::stop_idx_t> const subrange) {
          auto const stop = fr[subrange.from_ - fr.stop_range_.from_];
          auto const fragments =
              tags.id_fragments(tt, stop, n::event_type::kDep);
          if (fragments.tag_ != tag) {
            return;
          }

          add_lookup_entry(
              lookup, ambiguous,
              make_lookup_key(stop.get_route_id(n::event_type::kDep),
                              fragments.start_date_, fragments.start_time_),
              fragments.trip_id_);
        });
      }
    }
  }

  return lookup;
}

bool rewrite_gtfsrt_trip_ids(transit_realtime::FeedMessage& feed,
                             gtfsrt_trip_id_lookup_t const& lookup) {
  auto changed = false;
  for (auto& entity : *feed.mutable_entity()) {
    if (entity.has_trip_update()) {
      changed |= rewrite_trip_descriptor(
          entity.mutable_trip_update()->mutable_trip(), lookup);
    }

    if (entity.has_alert()) {
      for (auto& informed : *entity.mutable_alert()->mutable_informed_entity()) {
        if (informed.has_trip()) {
          changed |= rewrite_trip_descriptor(informed.mutable_trip(), lookup);
        }
      }
    }
  }
  return changed;
}

}  // namespace motis
