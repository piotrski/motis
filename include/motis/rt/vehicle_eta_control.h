#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "nigiri/types.h"

#include "motis/config.h"

namespace motis {

class vehicle_eta_runtime_control {
public:
  using mode = config::timetable::vehicle_eta::mode;

  explicit vehicle_eta_runtime_control(std::filesystem::path);

  // Missing or invalid files keep the last valid state. Returns true only
  // after a complete version-1 document has been accepted.
  [[nodiscard]] bool reload();
  [[nodiscard]] mode resolve(std::string_view feed,
                             nigiri::clasz,
                             mode static_mode) const;
  [[nodiscard]] mode resolve(config const&,
                             std::string_view feed,
                             nigiri::clasz) const;
  [[nodiscard]] bool enabled(config const&) const;

private:
  struct override {
    std::string feed_;
    std::optional<std::vector<nigiri::clasz>> modes_;
    mode state_{mode::off};
  };
  struct state {
    bool force_off_{false};
    std::vector<override> overrides_;
  };

  std::filesystem::path path_;
  std::optional<state> last_good_;
};

}  // namespace motis
