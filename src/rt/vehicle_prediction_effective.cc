#include "motis/rt/vehicle_prediction_effective.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

namespace motis {
namespace {

std::optional<timing_candidate_rejection_reason> gps_rejection(
    timing_source_candidate const& gps,
    std::int64_t const now,
    effective_vehicle_prediction_policy const& policy,
    double const min_gps_confidence) {
  if (gps.source_ != vehicle_prediction_source::kGps ||
      !gps.physically_reachable_ || gps.predictions_.empty()) {
    return timing_candidate_rejection_reason::kPhysicallyUnreachable;
  }
  if (gps.confidence_ < min_gps_confidence) {
    return timing_candidate_rejection_reason::kLowConfidence;
  }
  if (gps.reference_timestamp_seconds_ > now ||
      now - gps.reference_timestamp_seconds_ > policy.max_gps_age_seconds_) {
    return timing_candidate_rejection_reason::kStale;
  }
  return std::nullopt;
}

bool usable_provider(timing_source_candidate const& provider,
                     std::int64_t const now) {
  constexpr auto kMaxFutureSkew = std::int64_t{60};
  auto const ceiling =
      now > std::numeric_limits<std::int64_t>::max() - kMaxFutureSkew
          ? std::numeric_limits<std::int64_t>::max()
          : now + kMaxFutureSkew;
  return provider.source_ == vehicle_prediction_source::kProvider &&
         provider.physically_reachable_ && !provider.predictions_.empty() &&
         provider.reference_timestamp_seconds_ <= ceiling;
}

vehicle_prediction_selection choose(
    timing_source_candidate const& candidate,
    vehicle_prediction_selection_reason const reason,
    std::optional<timing_candidate_rejection_reason> const gps_rejection =
        std::nullopt) {
  return {.source_ = candidate.source_,
          .reason_ = reason,
          .predictions_ = candidate.predictions_,
          .diagnostics_ = {.gps_rejection_ = gps_rejection,
                           .selected_confidence_ = candidate.confidence_}};
}

vehicle_prediction_selection select_effective_vehicle_prediction(
    vehicle_prediction_selection_input const& input,
    effective_vehicle_prediction_policy const& policy,
    double const min_gps_confidence) {
  auto const provider_ok =
      input.provider_.has_value() &&
      usable_provider(*input.provider_, input.now_seconds_);
  auto rejection = std::optional<timing_candidate_rejection_reason>{};
  if (input.gps_.has_value()) {
    rejection = gps_rejection(*input.gps_, input.now_seconds_, policy,
                              min_gps_confidence);
  }
  auto const gps_ok = input.gps_.has_value() && !rejection.has_value();
  auto const gps_recent_enough =
      !provider_ok || (input.gps_.has_value() &&
                       input.gps_->reference_timestamp_seconds_ >=
                           input.provider_->reference_timestamp_seconds_ -
                               policy.provider_timestamp_tolerance_seconds_);
  if (gps_ok && gps_recent_enough) {
    return choose(*input.gps_, vehicle_prediction_selection_reason::kGpsOnly);
  }
  if (gps_ok && !gps_recent_enough) {
    rejection = timing_candidate_rejection_reason::kTimestampNotComparable;
  }
  if (provider_ok) {
    return choose(*input.provider_,
                  vehicle_prediction_selection_reason::kProviderOnly,
                  rejection);
  }
  return {.diagnostics_ = {.gps_rejection_ = rejection}};
}

}  // namespace

vehicle_prediction_selection select_effective_vehicle_prediction(
    vehicle_prediction_selection_input const& input,
    effective_vehicle_prediction_policy const& policy) {
  return select_effective_vehicle_prediction(input, policy,
                                             policy.min_gps_confidence_);
}

vehicle_prediction_selection
effective_vehicle_prediction_selection_state::select(
    vehicle_prediction_selection_input const& input,
    effective_vehicle_prediction_policy const& policy,
    std::optional<nigiri::interval<nigiri::stop_idx_t>> const& stop_range) {
  expire(input.now_seconds_, policy);
  auto const existing = std::ranges::find_if(entries_, [&](entry const& item) {
    return item.transport_ == input.transport_ &&
           item.stop_range_ == stop_range;
  });
  auto const continue_gps =
      existing != end(entries_) &&
      existing->selected_ == vehicle_prediction_source::kGps;
  auto selection = select_effective_vehicle_prediction(
      input, policy,
      continue_gps ? policy.min_selected_gps_confidence_
                   : policy.min_gps_confidence_);
  if (continue_gps && selection.source_ == vehicle_prediction_source::kGps &&
      input.gps_.has_value() &&
      input.gps_->confidence_ < policy.min_gps_confidence_) {
    selection.reason_ = vehicle_prediction_selection_reason::kSourceHysteresis;
  }
  if (input.transport_ == nigiri::transport::invalid()) {
    return selection;
  }
  if (existing != end(entries_)) {
    selection.diagnostics_.source_transition_ =
        existing->selected_ != selection.source_;
    existing->selected_ = selection.source_;
    existing->last_seen_seconds_ = input.now_seconds_;
  } else {
    entries_.push_back({.transport_ = input.transport_,
                        .stop_range_ = stop_range,
                        .selected_ = selection.source_,
                        .last_seen_seconds_ = input.now_seconds_});
  }
  return selection;
}

void effective_vehicle_prediction_selection_state::expire(
    std::int64_t const now_seconds,
    effective_vehicle_prediction_policy const& policy) {
  std::erase_if(entries_, [&](entry const& item) {
    auto const last_seen = item.last_seen_seconds_;
    return policy.selection_state_ttl_seconds_ <= 0 ||
           now_seconds < last_seen ||
           now_seconds - last_seen > policy.selection_state_ttl_seconds_;
  });
}

void effective_vehicle_prediction_selection_state::erase(
    nigiri::transport const& transport,
    std::optional<nigiri::interval<nigiri::stop_idx_t>> const& stop_range) {
  std::erase_if(entries_, [&](entry const& item) {
    return item.transport_ == transport && item.stop_range_ == stop_range;
  });
}

std::int64_t round_delay_minutes(std::int64_t const delay_seconds,
                                 std::optional<std::int64_t> const previous,
                                 std::int64_t const deadband_seconds) {
  auto rounded = delay_seconds / 60;
  auto const remainder = delay_seconds % 60;
  if (remainder >= 30) {
    ++rounded;
  } else if (remainder <= -30) {
    --rounded;
  }
  auto const adjacent = previous.has_value() &&
                        ((rounded > *previous &&
                          *previous !=
                              std::numeric_limits<std::int64_t>::max() &&
                          rounded == *previous + 1) ||
                         (rounded < *previous &&
                          *previous !=
                              std::numeric_limits<std::int64_t>::min() &&
                          rounded == *previous - 1));
  if (!adjacent || deadband_seconds <= 0) {
    return rounded;
  }
  auto const boundary =
      rounded > *previous ? rounded * 60 - 30 : rounded * 60 + 30;
  return std::llabs(delay_seconds - boundary) < deadband_seconds ? *previous
                                                                 : rounded;
}

}  // namespace motis
