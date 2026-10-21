#include "motis/vehicle_stream/protocol.h"

#include <algorithm>
#include <charconv>
#include <unordered_set>
#include <utility>

#include "boost/json.hpp"
#include "boost/url/url_view.hpp"

#include "motis/parse_location.h"
#include "motis/rt/vehicle_matching.h"

namespace json = boost::json;

namespace motis::vehicle_stream {

namespace {

std::optional<geo::latlng> parse_coordinate(std::string_view const s) {
  auto const location = parse_location(s);
  if (!location.has_value()) {
    return std::nullopt;
  }
  return location->pos_;
}

std::optional<std::int64_t> parse_int(std::string_view const s) {
  auto value = std::int64_t{};
  auto const [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc{} || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

parse_result finish(subscription sub) {
  if (sub.max_age_.has_value() && *sub.max_age_ < 0) {
    return std::string{"maxAge must be greater than or equal to zero"};
  }
  if (sub.id_.size() > 64U) {
    return std::string{"id too long"};
  }
  return sub;
}

std::int64_t to_ms(std::int64_t const s) { return s * 1000; }

}  // namespace

std::string lang_key(std::optional<std::vector<std::string>> const& lang) {
  if (!lang.has_value()) {
    return {};
  }
  auto key = std::string{};
  for (auto const& l : *lang) {
    key.append(l).push_back(',');
  }
  return key;
}

parse_result parse_subscription(json::object const& o) {
  auto sub = subscription{};

  auto const string_field = [&](char const* name) -> std::optional<std::string> {
    auto const it = o.find(name);
    if (it == o.end() || !it->value().is_string()) {
      return std::nullopt;
    }
    return std::string{it->value().as_string()};
  };

  auto const min = string_field("min");
  auto const max = string_field("max");
  if (!min.has_value() || !max.has_value()) {
    return std::string{"subscribe requires min and max"};
  }
  auto const min_pos = parse_coordinate(*min);
  auto const max_pos = parse_coordinate(*max);
  if (!min_pos.has_value() || !max_pos.has_value()) {
    return std::string{"min/max not a coordinate"};
  }
  sub.min_ = *min_pos;
  sub.max_ = *max_pos;

  if (auto const it = o.find("id"); it != o.end()) {
    if (it->value().is_string()) {
      sub.id_ = std::string{it->value().as_string()};
    } else if (it->value().is_int64()) {
      sub.id_ = std::to_string(it->value().as_int64());
    } else {
      return std::string{"id must be a string"};
    }
  }

  if (auto const it = o.find("language"); it != o.end()) {
    if (!it->value().is_array()) {
      return std::string{"language must be an array of strings"};
    }
    auto languages = std::vector<std::string>{};
    for (auto const& l : it->value().as_array()) {
      if (!l.is_string()) {
        return std::string{"language must be an array of strings"};
      }
      languages.emplace_back(l.as_string());
    }
    sub.language_ = std::move(languages);
  }

  if (auto const it = o.find("maxAge"); it != o.end()) {
    if (!it->value().is_int64()) {
      return std::string{"maxAge must be an integer"};
    }
    sub.max_age_ = it->value().as_int64();
  }

  if (auto const it = o.find("includeUnmatched"); it != o.end()) {
    sub.include_unmatched_ = it->value().is_bool() && it->value().as_bool();
  }
  if (auto const it = o.find("resync"); it != o.end()) {
    sub.resync_ = it->value().is_bool() && it->value().as_bool();
  }

  return finish(std::move(sub));
}

std::optional<parse_result> parse_subscription_query(
    std::string_view const target) {
  auto const url = boost::urls::url_view{target};
  auto const params = url.params();
  auto const min = params.find("min");
  auto const max = params.find("max");
  if (min == params.end() && max == params.end()) {
    return std::nullopt;
  }
  if (min == params.end() || max == params.end() || !(*min).has_value ||
      !(*max).has_value) {
    return parse_result{std::string{"subscribe requires min and max"}};
  }

  auto o = json::object{};
  o["min"] = (*min).value;
  o["max"] = (*max).value;
  for (auto const& p : params) {
    if (!p.has_value) {
      continue;
    }
    if (p.key == "id") {
      o["id"] = p.value;
    } else if (p.key == "language") {
      auto& languages = o["language"];
      if (!languages.is_array()) {
        languages = json::array{};
      }
      languages.as_array().emplace_back(p.value);
    } else if (p.key == "maxAge") {
      auto const value = parse_int(p.value);
      if (!value.has_value()) {
        return parse_result{std::string{"maxAge must be an integer"}};
      }
      o["maxAge"] = *value;
    } else if (p.key == "includeUnmatched") {
      o["includeUnmatched"] = p.value == "true";
    }
  }
  return parse_subscription(o);
}

std::vector<entry const*> select_vehicles(subscription const& sub,
                                 cycle const& c,
                                 std::int64_t const now,
                                 limits const& l) {
  auto const viewport =
      vehicle_positions::vehicle_viewport{.min_ = sub.min_, .max_ = sub.max_};
  auto const cutoff = vehicle_matching::freshness_cutoff(
      now, sub.max_age_.value_or(l.default_max_age_));
  auto selected = std::vector<entry const*>{};
  for (auto const& e : c.entries_) {
    if ((!sub.include_unmatched_ && !e.matched_) ||
        !viewport.contains(e.vehicle_->reported_position_.pos_) ||
        !vehicle_matching::is_fresh(*e.vehicle_, cutoff, now)) {
      continue;
    }
    selected.push_back(&e);
  }
  return selected;
}

message next_message(connection_state& s,
                     cycle const& c,
                     std::int64_t const now,
                     limits const& l) {
  auto const& sub = s.sub_.value();
  auto const selected = select_vehicles(sub, c, now, l);

  auto out = json::object{};
  auto msg = message{};

  if (selected.size() > l.max_vehicles_) {
    out["type"] = "error";
    out["sub"] = sub.id_;
    out["message"] = "too many vehicles, zoom in";
    msg.json_ = json::serialize(out);
    return msg;
  }

  out["seq"] = ++s.seq_;
  out["sub"] = sub.id_;
  out["cycle"] = c.id_;
  out["publishedAt"] = c.published_at_ms_;
  out["serverTime"] = to_ms(now);
  out["updateInterval"] = l.update_interval_;

  if (!s.snapshot_sent_ || sub.resync_) {
    out["type"] = "snapshot";
    auto vehicles = json::array{};
    vehicles.reserve(selected.size());
    s.sent_.clear();
    for (auto const* e : selected) {
      vehicles.emplace_back(*e->json_);
      s.sent_.emplace(e->key_, e->json_);
    }
    out["vehicles"] = std::move(vehicles);
    s.snapshot_sent_ = true;
    s.sub_->resync_ = false;
    msg.snapshot_ = true;
    msg.upserts_ = selected.size();
    msg.json_ = json::serialize(out);
    return msg;
  }

  out["type"] = "delta";
  auto upserts = json::array{};
  auto patches = json::array{};
  auto removes = json::array{};
  auto seen = std::unordered_set<std::string_view>{};
  seen.reserve(selected.size());

  for (auto const* e : selected) {
    seen.emplace(e->key_);
    auto const it = s.sent_.find(e->key_);
    if (it == end(s.sent_)) {
      upserts.emplace_back(*e->json_);
      s.sent_.emplace(e->key_, e->json_);
      continue;
    }
    if (it->second == e->json_ || *it->second == *e->json_) {
      it->second = e->json_;
      continue;
    }

    auto const& before = *it->second;
    auto const& after = *e->json_;
    auto set = json::object{};
    auto unset = json::array{};
    for (auto const& [k, v] : after) {
      auto const prev = before.find(k);
      if (prev == before.end() || prev->value() != v) {
        set[k] = v;
      }
    }
    for (auto const& [k, v] : before) {
      if (!after.contains(k)) {
        unset.emplace_back(k);
      }
    }
    auto patch = json::object{};
    patch["key"] = e->key_;
    patch["set"] = std::move(set);
    if (!unset.empty()) {
      patch["unset"] = std::move(unset);
    }
    patches.emplace_back(std::move(patch));
    it->second = e->json_;
  }

  for (auto it = begin(s.sent_); it != end(s.sent_);) {
    if (seen.contains(it->first)) {
      ++it;
    } else {
      removes.emplace_back(it->first);
      it = s.sent_.erase(it);
    }
  }

  msg.upserts_ = upserts.size();
  msg.patches_ = patches.size();
  msg.removes_ = removes.size();
  out["upserts"] = std::move(upserts);
  out["patches"] = std::move(patches);
  out["removes"] = std::move(removes);
  msg.json_ = json::serialize(out);
  return msg;
}

}  // namespace motis::vehicle_stream
