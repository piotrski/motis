#include "motis/rt/vehicle_eta_control.h"

#include <fstream>
#include <iterator>

#include "boost/json.hpp"

#include "nigiri/clasz.h"

namespace motis {
namespace {

using mode = config::timetable::vehicle_eta::mode;

std::optional<mode> parse_mode(std::string_view const value) {
  if (value == "off") {
    return mode::off;
  }
  if (value == "shadow") {
    return mode::shadow;
  }
  if (value == "effective") {
    return mode::effective;
  }
  return std::nullopt;
}

}  // namespace

vehicle_eta_runtime_control::vehicle_eta_runtime_control(
    std::filesystem::path path)
    : path_{std::move(path)} {}

bool vehicle_eta_runtime_control::reload() {
  try {
    auto input = std::ifstream{path_};
    if (!input) {
      return false;
    }
    auto const text = std::string{std::istreambuf_iterator<char>{input}, {}};
    auto const root = boost::json::parse(text).as_object();
    if (root.at("version").as_int64() != 1) {
      return false;
    }
    auto next = state{.force_off_ = root.at("forceOff").as_bool(),
                      .overrides_ = {}};
    auto const& feeds = root.at("feeds").as_array();
    next.overrides_.reserve(feeds.size());
    for (auto const& value : feeds) {
      auto const& object = value.as_object();
      auto const parsed = parse_mode(object.at("state").as_string());
      if (!parsed.has_value()) {
        return false;
      }
      auto item = override{.feed_ = object.at("feedId").as_string().c_str(),
                           .modes_ = std::nullopt,
                           .state_ = *parsed};
      if (auto const* modes = object.if_contains("modes")) {
        item.modes_ = std::vector<nigiri::clasz>{};
        for (auto const& entry : modes->as_array()) {
          auto const& name = entry.as_string();
          item.modes_->push_back(parse_vehicle_eta_clasz(
              std::string_view{name.data(), name.size()}));
        }
        if (item.modes_->empty()) {
          return false;
        }
      }
      next.overrides_.emplace_back(std::move(item));
    }
    last_good_ = std::move(next);
    return true;
  } catch (...) {
    return false;
  }
}

vehicle_eta_runtime_control::mode vehicle_eta_runtime_control::resolve(
    std::string_view const feed,
    nigiri::clasz const transit_mode,
    mode const static_mode) const {
  if (!last_good_.has_value()) {
    return static_mode;
  }
  if (last_good_->force_off_) {
    return mode::off;
  }
  for (auto const& item : last_good_->overrides_) {
    if (item.feed_ == feed && item.modes_.has_value() &&
        std::ranges::contains(*item.modes_, transit_mode)) {
      return item.state_;
    }
  }
  for (auto const& item : last_good_->overrides_) {
    if (item.feed_ == feed && !item.modes_.has_value()) {
      return item.state_;
    }
  }
  return static_mode;
}

vehicle_eta_runtime_control::mode vehicle_eta_runtime_control::resolve(
    config const& c,
    std::string_view const feed,
    nigiri::clasz const transit_mode) const {
  return resolve(feed, transit_mode, c.vehicle_eta_mode(feed, transit_mode));
}

bool vehicle_eta_runtime_control::enabled(config const& c) const {
  if (!c.timetable_ || !c.timetable_->vehicle_eta_) {
    return false;
  }
  for (auto const& [feed, _] : c.timetable_->datasets_) {
    for (auto i = 0U; i != nigiri::kNumClasses; ++i) {
      if (resolve(c, feed, static_cast<nigiri::clasz>(i)) != mode::off) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace motis
