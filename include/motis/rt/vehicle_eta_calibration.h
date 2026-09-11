#pragma once

#include <compare>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
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
    std::uintmax_t max_pending_bytes_{1024U * 1024U};
    std::int64_t retention_seconds_{7 * 24 * 60 * 60};
  };

  struct ingest_result {
    std::size_t pending_entries_{};
    std::size_t completed_entries_{};
    std::size_t queued_records_{};
    std::uintmax_t queued_bytes_{};
    std::size_t dropped_records_{};
    std::uintmax_t dropped_bytes_{};
    std::size_t persistence_failed_records_{};
    std::uintmax_t persistence_failed_bytes_{};
  };

  explicit vehicle_eta_calibration(std::filesystem::path directory);
  vehicle_eta_calibration(std::filesystem::path directory, limits);
  ~vehicle_eta_calibration();

  [[nodiscard]] ingest_result ingest(
      std::span<vehicle_prediction_cycle_result const>,
      std::int64_t now_seconds,
      std::int64_t update_interval_seconds);

  void flush() const;

  // Report aggregation is intentionally explicit: it reads all retained
  // calibration records and must not run on the realtime publication path.
  void generate_report() const;

  // Reads complete JSONL records and tolerates one partial final record.
  [[nodiscard]] static std::vector<boost::json::value> read_records(
      std::filesystem::path const&);

private:
  struct persistence;
  struct event_key {
    std::string feed_;
    std::string trip_id_;
    unsigned stop_sequence_{};
    std::int64_t scheduled_seconds_{};
    std::string bucket_;

    auto operator<=>(event_key const&) const = default;
  };
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

  std::filesystem::path directory_;
  std::unique_ptr<persistence> persistence_;
  std::map<event_key, forecast> forecasts_;
  std::map<event_key, std::int64_t> completed_;
};

}  // namespace motis
