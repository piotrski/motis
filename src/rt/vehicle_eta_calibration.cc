#include "motis/rt/vehicle_eta_calibration.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "boost/json.hpp"

#include "nigiri/clasz.h"
#include "nigiri/logging.h"

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

void rotate_and_prune(fs::path const& directory,
                      vehicle_eta_calibration::limits const& limits,
                      std::int64_t const now,
                      std::uintmax_t const incoming_bytes) {
  auto const current = directory / "calibration.jsonl";
  auto ec = std::error_code{};
  if (fs::exists(current, ec) && fs::file_size(current, ec) != 0U &&
      fs::file_size(current, ec) + incoming_bytes > limits.max_file_bytes_) {
    fs::rename(current,
               directory / ("calibration-" + std::to_string(now) + ".jsonl"),
               ec);
  }
  for (auto const& entry : fs::directory_iterator{directory, ec}) {
    if (entry.path() == current || entry.path().extension() != ".jsonl") {
      continue;
    }
    auto const modified = fs::last_write_time(entry.path(), ec);
    if (ec) {
      continue;
    }
    auto const age = fs::file_time_type::clock::now() - modified;
    if (age > std::chrono::seconds{limits.retention_seconds_}) {
      fs::remove(entry.path(), ec);
    }
  }
}

}  // namespace

struct vehicle_eta_calibration::persistence {
  struct batch {
    std::vector<std::string> lines_;
    std::int64_t now_{};
    std::uintmax_t bytes_{};
  };

  persistence(fs::path directory, limits const policy)
      : directory_{std::move(directory)},
        limits_{policy},
        worker_{[this]() { run(); }} {}

  ~persistence() {
    {
      auto lock = std::lock_guard{mutex_};
      closing_ = true;
    }
    ready_.notify_one();
    worker_.join();
  }

  bool enqueue(batch value) {
    auto lock = std::lock_guard{mutex_};
    if (value.bytes_ > limits_.max_pending_bytes_ ||
        outstanding_bytes_ > limits_.max_pending_bytes_ - value.bytes_) {
      return false;
    }
    outstanding_bytes_ += value.bytes_;
    queue_.emplace_back(std::move(value));
    ready_.notify_one();
    return true;
  }

  void flush() {
    auto lock = std::unique_lock{mutex_};
    drained_.wait(lock, [&]() { return outstanding_bytes_ == 0U; });
  }

  std::pair<std::size_t, std::uintmax_t> failures() const {
    return {failed_records_.load(std::memory_order_relaxed),
            failed_bytes_.load(std::memory_order_relaxed)};
  }

private:
  void run() {
    while (true) {
      auto value = batch{};
      {
        auto lock = std::unique_lock{mutex_};
        ready_.wait(lock, [&]() { return closing_ || !queue_.empty(); });
        if (queue_.empty()) {
          return;
        }
        value = std::move(queue_.front());
        queue_.pop_front();
      }
      try {
        fs::create_directories(directory_);
        rotate_and_prune(directory_, limits_, value.now_, value.bytes_);
        auto output = std::ofstream{directory_ / "calibration.jsonl",
                                    std::ios::app | std::ios::binary};
        for (auto const& line : value.lines_) {
          output << line << '\n';
        }
        output.flush();
        if (!output) {
          throw std::runtime_error{"unable to persist calibration batch"};
        }
      } catch (std::exception const& e) {
        failed_records_.fetch_add(value.lines_.size(),
                                  std::memory_order_relaxed);
        failed_bytes_.fetch_add(value.bytes_, std::memory_order_relaxed);
        nigiri::log(nigiri::log_lvl::error, "motis.rt",
                    "CALIBRATION PERSISTENCE ERROR: {}", e.what());
      } catch (...) {
        failed_records_.fetch_add(value.lines_.size(),
                                  std::memory_order_relaxed);
        failed_bytes_.fetch_add(value.bytes_, std::memory_order_relaxed);
        nigiri::log(nigiri::log_lvl::error, "motis.rt",
                    "CALIBRATION PERSISTENCE ERROR: unknown");
      }
      {
        auto lock = std::lock_guard{mutex_};
        outstanding_bytes_ -= value.bytes_;
      }
      drained_.notify_all();
    }
  }

  fs::path directory_;
  limits limits_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::condition_variable drained_;
  std::deque<batch> queue_;
  std::uintmax_t outstanding_bytes_{};
  bool closing_{};
  std::thread worker_;
  std::atomic<std::size_t> failed_records_{};
  std::atomic<std::uintmax_t> failed_bytes_{};
};

vehicle_eta_calibration::vehicle_eta_calibration(fs::path directory)
    : vehicle_eta_calibration{std::move(directory), limits{}} {}

vehicle_eta_calibration::vehicle_eta_calibration(fs::path directory,
                                                 limits const policy)
    : directory_{std::move(directory)},
      persistence_{std::make_unique<persistence>(directory_, policy)} {}

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
      auto const key = event_key{
          .feed_ = candidate.feed_,
          .trip_id_ = candidate.trip_id_,
          .stop_sequence_ = prediction.static_stop_sequence_,
          .scheduled_seconds_ = prediction.scheduled_timestamp_seconds_,
          .bucket_ = std::string{*bucket}};
      if (completed_.contains(key) || forecasts_.contains(key)) {
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
          key,
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
      auto const first = event_key{
          .feed_ = candidate.feed_,
          .trip_id_ = candidate.trip_id_,
          .stop_sequence_ = passage.static_stop_sequence_,
          .scheduled_seconds_ = std::numeric_limits<std::int64_t>::min(),
          .bucket_ = {}};
      for (auto it = forecasts_.lower_bound(first);
           it != end(forecasts_) && it->first.feed_ == candidate.feed_ &&
           it->first.trip_id_ == candidate.trip_id_ &&
           it->first.stop_sequence_ == passage.static_stop_sequence_;) {
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
  auto batch = persistence::batch{.lines_ = {}, .now_ = now, .bytes_ = 0U};
  batch.lines_.reserve(records.size());
  for (auto const& record : records) {
    auto line = boost::json::serialize(record);
    batch.bytes_ += line.size() + 1U;
    batch.lines_.emplace_back(std::move(line));
  }
  auto const record_count = batch.lines_.size();
  auto const bytes = batch.bytes_;
  auto const queued =
      records.empty() || persistence_->enqueue(std::move(batch));
  auto const [failed_records, failed_bytes] = persistence_->failures();
  return {.pending_entries_ = forecasts_.size(),
          .completed_entries_ = completed_.size(),
          .queued_records_ = queued ? record_count : 0U,
          .queued_bytes_ = queued ? bytes : 0U,
          .dropped_records_ = queued ? 0U : record_count,
          .dropped_bytes_ = queued ? 0U : bytes,
          .persistence_failed_records_ = failed_records,
          .persistence_failed_bytes_ = failed_bytes};
}

void vehicle_eta_calibration::flush() const { persistence_->flush(); }

void vehicle_eta_calibration::generate_report() const {
  flush();
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
