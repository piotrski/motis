#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "boost/json/object.hpp"
#include "boost/json/value.hpp"

#include "motis/rt/vehicle_prediction_diagnostics.h"

namespace motis {

class vehicle_eta_calibration {
public:
  struct limits {
    std::uintmax_t max_file_bytes_{10U * 1024U * 1024U};
    std::int64_t retention_seconds_{7 * 24 * 60 * 60};
  };

  struct ingest_result {
    std::size_t pending_entries_{};
    std::size_t completed_entries_{};
    std::size_t appended_records_{};
    std::uintmax_t appended_bytes_{};
  };

  explicit vehicle_eta_calibration(std::filesystem::path directory);
  vehicle_eta_calibration(std::filesystem::path directory, limits);
  ~vehicle_eta_calibration();

  [[nodiscard]] ingest_result ingest(
      std::span<vehicle_prediction_cycle_result const>,
      std::int64_t now_seconds,
      std::int64_t update_interval_seconds);

  // Report aggregation is intentionally explicit: it reads all retained
  // calibration records and must not run on the realtime publication path.
  void generate_report() const;

  // Reads complete JSONL records and tolerates one partial final record.
  [[nodiscard]] static std::vector<boost::json::value> read_records(
      std::filesystem::path const&);

private:
  struct forecast {
    std::string feed_;
    std::string trip_id_;
    std::string mode_;
    std::string bucket_;
    unsigned stop_sequence_{};
    std::int64_t scheduled_seconds_{};
    std::int64_t captured_seconds_{};
    std::int64_t gps_seconds_{};
    std::optional<std::int64_t> provider_seconds_;
  };

  [[nodiscard]] std::uintmax_t append(std::span<boost::json::object const>,
                                      std::int64_t now_seconds);
  void rotate_and_prune(std::int64_t now_seconds,
                        std::uintmax_t incoming_bytes);

  std::filesystem::path directory_;
  limits limits_;
  std::map<std::string, forecast> forecasts_;
  std::map<std::string, std::int64_t> completed_;
};

}  // namespace motis
