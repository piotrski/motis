#include "motis/rt/vehicle_eta_calibration.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <string_view>

#include "boost/json.hpp"

#include "nigiri/clasz.h"

namespace fs = std::filesystem;

namespace motis {
namespace {

std::optional<std::string_view> horizon_bucket(std::int64_t const seconds) {
  if (seconds < 0 || seconds > 40 * 60) {
    return std::nullopt;
  }
  if (seconds <= 2 * 60) {
    return "0-2";
  }
  if (seconds <= 5 * 60) {
    return "2-5";
  }
  if (seconds <= 10 * 60) {
    return "5-10";
  }
  if (seconds <= 20 * 60) {
    return "10-20";
  }
  return "20-40";
}

std::string event_key_prefix(vehicle_prediction_cycle_result const& candidate,
                             unsigned const sequence) {
  return candidate.feed_ + "|" + candidate.trip_id_ + "|" +
         std::to_string(sequence) + "|";
}

std::string event_key(vehicle_prediction_cycle_result const& candidate,
                      unsigned const sequence,
                      std::int64_t const scheduled) {
  return event_key_prefix(candidate, sequence) + std::to_string(scheduled);
}

double percentile(std::vector<std::int64_t> values, double const quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::ranges::sort(values);
  auto const index = static_cast<std::size_t>(
      std::floor(quantile * static_cast<double>(values.size() - 1U)));
  return static_cast<double>(values[index]);
}

boost::json::array bootstrap_median_absolute_error_ci(
    std::map<std::string, std::vector<std::int64_t>> const& errors_by_trip) {
  if (errors_by_trip.empty()) {
    return {0.0, 0.0};
  }
  auto clusters = std::vector<std::vector<std::int64_t> const*>{};
  clusters.reserve(errors_by_trip.size());
  for (auto const& [trip, errors] : errors_by_trip) {
    (void)trip;
    clusters.push_back(&errors);
  }
  auto generator = std::mt19937{0x475053U};
  auto sample =
      std::uniform_int_distribution<std::size_t>{0U, clusters.size() - 1U};
  auto medians = std::vector<std::int64_t>{};
  medians.reserve(1'000U);
  for (auto repetition = 0U; repetition != 1'000U; ++repetition) {
    auto values = std::vector<std::int64_t>{};
    for (auto cluster = 0U; cluster != clusters.size(); ++cluster) {
      auto const& selected = *clusters[sample(generator)];
      for (auto const error : selected) {
        values.push_back(std::llabs(error));
      }
    }
    medians.push_back(static_cast<std::int64_t>(percentile(values, 0.5)));
  }
  return {percentile(medians, 0.025), percentile(medians, 0.975)};
}

}  // namespace

vehicle_eta_calibration::vehicle_eta_calibration(fs::path directory)
    : vehicle_eta_calibration{std::move(directory), limits{}} {}

vehicle_eta_calibration::vehicle_eta_calibration(fs::path directory,
                                                 limits const policy)
    : directory_{std::move(directory)}, limits_{policy} {}

vehicle_eta_calibration::~vehicle_eta_calibration() = default;

vehicle_eta_calibration::ingest_result vehicle_eta_calibration::ingest(
    std::span<vehicle_prediction_cycle_result const> const candidates,
    std::int64_t const now,
    std::int64_t const update_interval_seconds) {
  constexpr auto kForecastExpiryAfterScheduleSeconds = 40 * 60;
  std::erase_if(forecasts_, [&](auto const& entry) {
    return entry.second.scheduled_seconds_ +
               kForecastExpiryAfterScheduleSeconds <
           now;
  });
  std::erase_if(completed_, [&](auto const& entry) {
    return entry.second + kForecastExpiryAfterScheduleSeconds < now;
  });

  auto records = std::vector<boost::json::object>{};
  for (auto const& candidate : candidates) {
    if (!candidate.mode_.has_value() || !candidate.batch_.eligible()) {
      continue;
    }
    for (auto const& prediction : candidate.batch_.predictions_) {
      if (prediction.event_type_ != vehicle_prediction_event_type::kArrival) {
        continue;
      }
      auto const bucket =
          horizon_bucket(prediction.predicted_timestamp_seconds_ - now);
      if (!bucket.has_value()) {
        continue;
      }
      auto const key = event_key(candidate, prediction.static_stop_sequence_,
                                 prediction.scheduled_timestamp_seconds_);
      auto const unique = key + "|" + std::string{*bucket};
      if (completed_.contains(unique) || forecasts_.contains(unique)) {
        continue;
      }
      auto const provider = std::ranges::find_if(
          candidate.provider_predictions_, [&](auto const& x) {
            return x.event_type_ == vehicle_prediction_event_type::kArrival &&
                   x.static_stop_sequence_ ==
                       prediction.static_stop_sequence_ &&
                   x.scheduled_timestamp_seconds_ ==
                       prediction.scheduled_timestamp_seconds_;
          });
      forecasts_.emplace(
          unique,
          forecast{
              .feed_ = candidate.feed_,
              .trip_id_ = candidate.trip_id_,
              .mode_ = std::string{nigiri::to_str(*candidate.mode_)},
              .bucket_ = std::string{*bucket},
              .stop_sequence_ = prediction.static_stop_sequence_,
              .scheduled_seconds_ = prediction.scheduled_timestamp_seconds_,
              .captured_seconds_ = now,
              .gps_seconds_ = prediction.predicted_timestamp_seconds_,
              .provider_seconds_ =
                  provider == end(candidate.provider_predictions_)
                      ? std::nullopt
                      : std::optional{provider->predicted_timestamp_seconds_}});
    }

    for (auto const& passage : candidate.batch_.observed_passages_) {
      if (passage.uncertainty_seconds_ > 2 * update_interval_seconds) {
        continue;
      }
      auto const prefix =
          event_key_prefix(candidate, passage.static_stop_sequence_);
      for (auto it = forecasts_.lower_bound(prefix);
           it != end(forecasts_) && it->first.starts_with(prefix);) {
        auto const& forecast = it->second;
        if (passage.observed_timestamp_seconds_ <= forecast.captured_seconds_) {
          ++it;
          continue;
        }
        auto record = boost::json::object{
            {"version", 1},
            {"feed", forecast.feed_},
            {"mode", forecast.mode_},
            {"tripInstance", forecast.trip_id_},
            {"stopSequence", forecast.stop_sequence_},
            {"scheduledTime", forecast.scheduled_seconds_},
            {"horizon", forecast.bucket_},
            {"capturedTime", forecast.captured_seconds_},
            {"passageTime", passage.observed_timestamp_seconds_},
            {"uncertaintySeconds", passage.uncertainty_seconds_},
            {"gpsErrorSeconds",
             forecast.gps_seconds_ - passage.observed_timestamp_seconds_}};
        if (forecast.provider_seconds_.has_value()) {
          record["providerErrorSeconds"] =
              *forecast.provider_seconds_ - passage.observed_timestamp_seconds_;
        }
        records.emplace_back(std::move(record));
        completed_.emplace(it->first, forecast.scheduled_seconds_);
        it = forecasts_.erase(it);
      }
    }
  }
  auto const appended_bytes =
      records.empty() ? std::uintmax_t{0U} : append(records, now);
  return {.pending_entries_ = forecasts_.size(),
          .completed_entries_ = completed_.size(),
          .appended_records_ = records.size(),
          .appended_bytes_ = appended_bytes};
}

std::uintmax_t vehicle_eta_calibration::append(
    std::span<boost::json::object const> const records,
    std::int64_t const now) {
  fs::create_directories(directory_);
  auto serialized = std::vector<std::string>{};
  serialized.reserve(records.size());
  auto incoming_bytes = std::uintmax_t{0U};
  for (auto const& record : records) {
    auto line = boost::json::serialize(record);
    incoming_bytes += line.size() + 1U;
    serialized.emplace_back(std::move(line));
  }
  rotate_and_prune(now, incoming_bytes);
  auto output = std::ofstream{directory_ / "calibration.jsonl",
                              std::ios::app | std::ios::binary};
  for (auto const& line : serialized) {
    output << line << '\n';
  }
  output.flush();
  return incoming_bytes;
}

void vehicle_eta_calibration::generate_report() const {
  fs::create_directories(directory_);
  struct aggregate {
    std::vector<std::int64_t> gps_errors_;
    std::vector<std::int64_t> provider_errors_;
    std::vector<std::int64_t> uncertainty_;
    std::set<std::string> trips_;
    std::map<std::string, std::vector<std::int64_t>> gps_errors_by_trip_;
    std::map<std::string, std::vector<std::int64_t>> provider_errors_by_trip_;
  };
  auto groups = std::map<std::string, aggregate>{};
  auto ec = std::error_code{};
  for (auto const& entry : fs::directory_iterator{directory_, ec}) {
    if (entry.path().extension() != ".jsonl") {
      continue;
    }
    for (auto const& value : read_records(entry.path())) {
      auto const& record = value.as_object();
      auto const key = std::string{record.at("feed").as_string()} + "|" +
                       std::string{record.at("mode").as_string()} + "|" +
                       std::string{record.at("horizon").as_string()};
      auto& group = groups[key];
      auto const trip = std::string{record.at("tripInstance").as_string()};
      auto const gps_error = record.at("gpsErrorSeconds").as_int64();
      group.gps_errors_.push_back(gps_error);
      group.gps_errors_by_trip_[trip].push_back(gps_error);
      if (auto const* provider = record.if_contains("providerErrorSeconds")) {
        group.provider_errors_.push_back(provider->as_int64());
        group.provider_errors_by_trip_[trip].push_back(provider->as_int64());
      }
      group.uncertainty_.push_back(record.at("uncertaintySeconds").as_int64());
      group.trips_.insert(trip);
    }
  }
  auto output_groups = boost::json::array{};
  for (auto const& [key, group] : groups) {
    auto const first = key.find('|');
    auto const second = key.find('|', first + 1U);
    auto absolute = [](std::vector<std::int64_t> values) {
      for (auto& value : values) {
        value = std::llabs(value);
      }
      return values;
    };
    auto const gps_absolute = absolute(group.gps_errors_);
    auto const provider_absolute = absolute(group.provider_errors_);
    auto const bias = [](std::vector<std::int64_t> const& values) {
      return values.empty()
                 ? 0.0
                 : static_cast<double>(std::accumulate(
                       begin(values), end(values), std::int64_t{0})) /
                       static_cast<double>(values.size());
    };
    output_groups.push_back(boost::json::object{
        {"feed", key.substr(0U, first)},
        {"mode", key.substr(first + 1U, second - first - 1U)},
        {"horizon", key.substr(second + 1U)},
        {"uniqueEvents", group.gps_errors_.size()},
        {"distinctTrips", group.trips_.size()},
        {"readyForReview",
         group.gps_errors_.size() >= 500U && group.trips_.size() >= 50U},
        {"pairedCoverage",
         group.gps_errors_.empty()
             ? 0.0
             : static_cast<double>(group.provider_errors_.size()) /
                   static_cast<double>(group.gps_errors_.size())},
        {"gpsMedianAbsoluteErrorSeconds", percentile(gps_absolute, 0.5)},
        {"gpsMedianAbsoluteError95CiSeconds",
         bootstrap_median_absolute_error_ci(group.gps_errors_by_trip_)},
        {"gpsP90AbsoluteErrorSeconds", percentile(gps_absolute, 0.9)},
        {"gpsSignedBiasSeconds", bias(group.gps_errors_)},
        {"providerMedianAbsoluteErrorSeconds",
         percentile(provider_absolute, 0.5)},
        {"providerMedianAbsoluteError95CiSeconds",
         bootstrap_median_absolute_error_ci(group.provider_errors_by_trip_)},
        {"providerP90AbsoluteErrorSeconds", percentile(provider_absolute, 0.9)},
        {"providerSignedBiasSeconds", bias(group.provider_errors_)},
        {"medianUncertaintySeconds", percentile(group.uncertainty_, 0.5)}});
  }
  auto const temporary = directory_ / "report.json.tmp";
  {
    auto output = std::ofstream{temporary, std::ios::binary};
    output << boost::json::serialize(boost::json::object{
        {"version", 1}, {"groups", std::move(output_groups)}});
    output.flush();
  }
  fs::rename(temporary, directory_ / "report.json", ec);
}

void vehicle_eta_calibration::rotate_and_prune(
    std::int64_t const now, std::uintmax_t const incoming_bytes) {
  auto const current = directory_ / "calibration.jsonl";
  auto ec = std::error_code{};
  if (fs::exists(current, ec) && fs::file_size(current, ec) != 0U &&
      fs::file_size(current, ec) + incoming_bytes > limits_.max_file_bytes_) {
    fs::rename(current,
               directory_ / ("calibration-" + std::to_string(now) + ".jsonl"),
               ec);
  }
  for (auto const& entry : fs::directory_iterator{directory_, ec}) {
    if (entry.path() == current || entry.path().extension() != ".jsonl") {
      continue;
    }
    auto const modified = fs::last_write_time(entry.path(), ec);
    if (ec) {
      continue;
    }
    auto const age = fs::file_time_type::clock::now() - modified;
    if (age > std::chrono::seconds{limits_.retention_seconds_}) {
      fs::remove(entry.path(), ec);
    }
  }
}

std::vector<boost::json::value> vehicle_eta_calibration::read_records(
    fs::path const& path) {
  auto input = std::ifstream{path};
  auto records = std::vector<boost::json::value>{};
  auto line = std::string{};
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    try {
      records.emplace_back(boost::json::parse(line));
    } catch (...) {
      if (!input.eof()) {
        throw;
      }
    }
  }
  return records;
}

}  // namespace motis
