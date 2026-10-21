#include "motis/rt/vehicle_prediction_diagnostics.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <optional>
#include <set>
#include <string_view>

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/frun.h"

#include "motis/config.h"
#include "motis/rt/vehicle_eta_control.h"
#include "motis/rt/vehicle_matching.h"
#include "motis/rt/vehicle_observation_history.h"
#include "motis/rt/vehicle_position.h"
#include "motis/rt/vehicle_prediction_continuation.h"
#include "motis/rt/vehicle_prediction_limits.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {
namespace {

auto configured_mode(config const& c,
                     vehicle_eta_runtime_control const* const control,
                     std::string_view const feed,
                     n::clasz const mode) {
  return control == nullptr ? c.vehicle_eta_mode(feed, mode)
                            : control->resolve(c, feed, mode);
}

bool feed_enabled(config const& c,
                  vehicle_eta_runtime_control const* const control,
                  std::string_view const feed) {
  for (auto i = 0U; i != n::kNumClasses; ++i) {
    if (configured_mode(c, control, feed, static_cast<n::clasz>(i)) !=
        config::timetable::vehicle_eta::mode::off) {
      return true;
    }
  }
  return false;
}

vehicle_key key_for(vehicle_positions::vehicle_position const& vehicle) {
  return vehicle.vehicle_.id_.has_value()
             ? vehicle_key{vehicle.feed_id_, *vehicle.vehicle_.id_,
                           vehicle_key_source::kVehicleDescriptor}
             : vehicle_key{vehicle.feed_id_, vehicle.entity_id_,
                           vehicle_key_source::kEntityId};
}

vehicle_trip_instance trip_for(
    vehicle_positions::vehicle_position const& vehicle) {
  return {.trip_id_ = vehicle.trip_.trip_id_,
          .start_date_ = vehicle.trip_.start_date_,
          .start_time_ = vehicle.trip_.start_time_};
}

vehicle_prediction_cycle_result rejected(
    std::string feed,
    vehicle_prediction_rejection_reason const reason,
    std::optional<std::int64_t> const latest_observation = std::nullopt) {
  auto result = vehicle_prediction_cycle_result{
      .feed_ = std::move(feed),
      .latest_vehicle_observation_timestamp_seconds_ = latest_observation};
  result.batch_.diagnostics_.rejection_ = reason;
  return result;
}

}  // namespace

std::vector<vehicle_prediction_cycle_result>
evaluate_vehicle_prediction_candidates(
    config const& c,
    tag_lookup const& tags,
    n::timetable const& tt,
    n::rt_timetable const* rtt,
    n::shapes_storage const* shapes,
    vehicle_positions::vehicle_position_store const& positions,
    vehicle_observation_history const& history,
    std::int64_t const now,
    vehicle_eta_runtime_control const* const control) {
  auto results = std::vector<vehicle_prediction_cycle_result>{};
  if (!c.timetable_ || !c.timetable_->vehicle_eta_ || shapes == nullptr) {
    return results;
  }
  auto const& history_policy = c.timetable_->vehicle_eta_->history_;
  auto engine = vehicle_prediction_engine{
      *shapes,
      vehicle_prediction_policy{
          .max_observation_age_seconds_ = history_policy.max_age_seconds_,
          .max_observation_gap_seconds_ =
              history_policy.max_observation_gap_seconds_,
          .early_departure_tolerance_seconds_ =
              c.timetable_->vehicle_eta_->selection_
                  .early_departure_tolerance_seconds_}};
  auto const cutoff =
      vehicle_matching::freshness_cutoff(now, history_policy.max_age_seconds_);
  auto enabled_feeds = std::set<std::string>{};
  for (auto const& position : positions.all()) {
    auto const feed = vehicle_matching::dataset_tag(position.feed_id_);
    if (feed_enabled(c, control, feed)) {
      enabled_feeds.emplace(feed);
    }
  }
  auto const feed_count = std::max(std::size_t{1U}, enabled_feeds.size());
  auto const evaluation_limit =
      std::max(std::size_t{1U},
               kMaxVehiclePredictionPositionsPerCycle / feed_count);
  auto const resolution_limit =
      std::max(std::size_t{1U},
               kMaxVehiclePredictionResolutionsPerCycle / feed_count);
  auto evaluated_positions = std::map<std::string, std::size_t>{};
  auto resolved_positions = std::map<std::string, std::size_t>{};
  for (auto const& position : positions.all()) {
    auto feed = std::string{vehicle_matching::dataset_tag(position.feed_id_)};
    if (!feed_enabled(c, control, feed)) {
      continue;
    }
    auto const latest_observation =
        position.reported_time_.value_or(position.ingested_time_);
    if (!position.trip_.trip_id_.has_value()) {
      results.push_back(rejected(
          std::move(feed), vehicle_prediction_rejection_reason::kMissingTripId,
          latest_observation));
      continue;
    }
    if (position.trip_.schedule_relationship_.has_value() &&
        *position.trip_.schedule_relationship_ != "SCHEDULED") {
      results.push_back(rejected(
          std::move(feed),
          vehicle_prediction_rejection_reason::kUnsupportedTripRelationship,
          latest_observation));
      continue;
    }
    auto const observations =
        history.observations(key_for(position), trip_for(position));
    if (observations.empty()) {
      results.push_back(
          rejected(std::move(feed),
                   vehicle_prediction_rejection_reason::kInsufficientHistory,
                   latest_observation));
      continue;
    }
    auto const has_fresh_history = std::ranges::any_of(
        observations, [&](vehicle_observation const& observation) {
          return (observation_time(observation) <= now ||
                  observation.ingested_time_ <= now) &&
                 std::min({observation_time(observation),
                           observation.ingested_time_, now}) >= cutoff;
        });
    if (!has_fresh_history) {
      results.push_back(rejected(
          std::move(feed), vehicle_prediction_rejection_reason::kStaleHistory,
          latest_observation));
      continue;
    }
    if (resolved_positions[feed]++ >= resolution_limit) {
      continue;
    }
    auto run = vehicle_matching::resolve_run(tags, tt, rtt, position);
    if (!run.has_value()) {
      results.push_back(rejected(
          std::move(feed), vehicle_prediction_rejection_reason::kUnresolvedTrip,
          latest_observation));
      continue;
    }
    if (!run->is_scheduled()) {
      results.push_back(
          rejected(std::move(feed),
                   vehicle_prediction_rejection_reason::kUnscheduledTrip,
                   latest_observation));
      continue;
    }
    auto const mode = (*run)[0].get_clasz(n::event_type::kDep);
    if (configured_mode(c, control, feed, mode) ==
        config::timetable::vehicle_eta::mode::off) {
      continue;
    }
    if (evaluated_positions[feed]++ >= evaluation_limit) {
      continue;
    }
    auto result = vehicle_prediction_cycle_result{
        .feed_ = std::move(feed),
        .trip_id_ = tags.id(tt, (*run)[0], n::event_type::kDep),
        .mode_ = mode,
        .trip_stop_range_ = run->stop_range_,
        .latest_vehicle_observation_timestamp_seconds_ = latest_observation};
    result.batch_ = engine.evaluate(*run, observations, now);
    if (result.batch_.eligible()) {
      auto const trip = (*run)[0].get_trip_idx(n::event_type::kDep);
      auto const encoded = tt.trip_stop_seq_numbers_[trip];
      auto const sequence_range = n::loader::gtfs::stop_seq_number_range{
          {encoded.data(), encoded.size()},
          static_cast<n::stop_idx_t>(run->stop_range_.size())};
      auto const sequences = std::vector<n::stop_idx_t>{begin(sequence_range),
                                                        end(sequence_range)};
      for (auto const& prediction : result.batch_.predictions_) {
        auto const stop_count =
            static_cast<n::stop_idx_t>(run->stop_range_.size());
        for (auto i = n::stop_idx_t{0U}; i != stop_count; ++i) {
          auto const event =
              prediction.event_type_ == vehicle_prediction_event_type::kArrival
                  ? n::event_type::kArr
                  : n::event_type::kDep;
          if ((i == 0U && event == n::event_type::kArr) ||
              (i + 1U == stop_count && event == n::event_type::kDep)) {
            continue;
          }
          auto const scheduled = to_seconds((*run)[i].scheduled_time(event));
          if (static_cast<unsigned>(sequences[i]) !=
                  prediction.static_stop_sequence_ ||
              scheduled != prediction.scheduled_timestamp_seconds_) {
            continue;
          }
          auto const provider = to_seconds((*run)[i].time(event));
          if (!run->is_rt() || provider == scheduled) {
            break;
          }
          result.provider_predictions_.push_back(
              {.static_stop_sequence_ = prediction.static_stop_sequence_,
               .event_type_ = prediction.event_type_,
               .scheduled_timestamp_seconds_ = scheduled,
               .predicted_timestamp_seconds_ = provider,
               .delay_seconds_ = provider - scheduled,
               .horizon_seconds_ = provider - now});
          auto const raw = prediction.predicted_timestamp_seconds_ - provider;
          result.provider_raw_error_seconds_.push_back(raw);
          result.provider_minute_error_.push_back(
              prediction.predicted_timestamp_seconds_ / 60 - provider / 60);
          break;
        }
      }
    }
    auto continuation = derive_vehicle_prediction_continuation(
        tt, tags, *run, vehicle_matching::dataset_tag(position.feed_id_),
        result.trip_id_, result.batch_);
    results.emplace_back(std::move(result));
    if (continuation.has_value()) {
      auto next = vehicle_prediction_cycle_result{
          .feed_ =
              std::string{vehicle_matching::dataset_tag(position.feed_id_)},
          .trip_id_ = continuation->trip_id_,
          .mode_ = continuation->mode_,
          .trip_stop_range_ = continuation->trip_stop_range_,
          .context_ = vehicle_prediction_context::kIncomingBlockLeg,
          .incoming_leg_provenance_ = continuation->provenance_,
          .latest_vehicle_observation_timestamp_seconds_ =
              continuation->reference_timestamp_seconds_};
      next.batch_.transport_ = continuation->transport_;
      next.batch_.candidate_reference_timestamp_seconds_ =
          continuation->reference_timestamp_seconds_;
      next.batch_.confidence_ =
          vehicle_prediction_confidence{.score_ = continuation->confidence_};
      next.batch_.predictions_ = std::move(continuation->predictions_);
      results.emplace_back(std::move(next));
    }
  }
  return results;
}

}  // namespace motis
