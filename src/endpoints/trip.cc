#include "motis/endpoints/trip.h"

#include <algorithm>
#include <chrono>

#include "date/date.h"

#include "net/not_found_exception.h"

#include "nigiri/routing/journey.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/timetable.h"

#include "motis/constants.h"
#include "motis/data.h"
#include "motis/gbfs/routing_data.h"
#include "motis/journey_to_response.h"
#include "motis/parse_location.h"
#include "motis/rt/vehicle_matching.h"
#include "motis/rt/vehicle_prediction_store.h"
#include "motis/server.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/clasz_to_mode.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis::ep {
namespace {

api::PredictionSourceEnum prediction_source(
    vehicle_prediction_source const source) {
  switch (source) {
    case vehicle_prediction_source::kProvider:
      return api::PredictionSourceEnum::PROVIDER;
    case vehicle_prediction_source::kGps: return api::PredictionSourceEnum::GPS;
    case vehicle_prediction_source::kSchedule:
      return api::PredictionSourceEnum::SCHEDULE;
  }
  std::unreachable();
}

std::optional<api::PredictionContextEnum> prediction_context(
    vehicle_prediction_diagnostic_entry const* const entry) {
  return entry != nullptr && entry->context_ ==
                                 vehicle_prediction_context::kIncomingBlockLeg
             ? std::optional{api::PredictionContextEnum::INCOMING_BLOCK_LEG}
             : std::nullopt;
}

std::string rfc3339(std::int64_t const timestamp_seconds) {
  return date::format("%FT%TZ", std::chrono::sys_seconds{
                                    std::chrono::seconds{timestamp_seconds}});
}

openapi::date_time_t date_time(std::int64_t const timestamp_seconds) {
  auto result = openapi::date_time_t{};
  result.time_ =
      std::chrono::sys_seconds{std::chrono::seconds{timestamp_seconds}};
  return result;
}

api::SelectedPrediction selected_prediction(
    n::timetable const& tt,
    tag_lookup const& tags,
    n::rt::frun const& run,
    n::stop_idx_t const stop_idx,
    n::event_type const event_type,
    vehicle_prediction_diagnostics_store const* const diagnostics) {
  auto const stop = run[stop_idx];
  auto const scheduled = to_seconds(stop.scheduled_time(event_type));
  auto const sequence = static_stop_sequence(
      tt, run.t_.t_idx_, stop.get_trip_idx(event_type), stop.stop_idx_);
  auto const* entry =
      diagnostics == nullptr || !sequence.has_value()
          ? nullptr
          : diagnostics->find_event(
                run.t_, tags.id(tt, stop, event_type), *sequence, scheduled,
                event_type == n::event_type::kArr
                    ? vehicle_prediction_event_type::kArrival
                    : vehicle_prediction_event_type::kDeparture);
  auto const selected = resolve_effective_prediction(
      run.is_rt(), scheduled, to_seconds(stop.time(event_type)), entry);
  return {.source_ = prediction_source(selected.source_),
          .time_ = rfc3339(selected.predicted_timestamp_seconds_),
          .scheduledTime_ = rfc3339(scheduled),
          .delaySeconds_ = selected.delay_seconds_,
          .confidence_ = selected.confidence_,
          .referenceTime_ = selected.reference_timestamp_seconds_.transform(
              [](auto const timestamp) { return rfc3339(timestamp); }),
          .eventType_ = event_type == n::event_type::kArr
                            ? api::PredictionEventTypeEnum::ARRIVAL
                            : api::PredictionEventTypeEnum::DEPARTURE,
          .context_ = prediction_context(entry)};
}

std::optional<api::IncomingLegPrediction> incoming_leg_prediction(
    n::transport const transport,
    std::string_view const trip_id,
    vehicle_prediction_diagnostics_store const* const diagnostics) {
  auto const* entry = diagnostics == nullptr
                          ? nullptr
                          : diagnostics->find_incoming_leg(transport, trip_id);
  if (entry == nullptr) {
    return std::nullopt;
  }
  auto const& incoming = *entry->incoming_leg_provenance_;
  return api::IncomingLegPrediction{
      .source_ = api::PredictionSourceEnum::GPS,
      .context_ = api::PredictionContextEnum::INCOMING_BLOCK_LEG,
      .incomingTripId_ = incoming.incoming_trip_id_,
      .nextTripId_ = incoming.next_trip_id_,
      .routeId_ = incoming.route_id_,
      .routeShortName_ = incoming.route_short_name_,
      .headsign_ = incoming.headsign_,
      .mode_ = to_mode(incoming.mode_, 6U),
      .expectedTerminalArrival_ =
          date_time(incoming.expected_terminal_arrival_seconds_),
      .nextScheduledDeparture_ =
          date_time(incoming.next_scheduled_departure_seconds_),
      .propagatedDelaySeconds_ = incoming.propagated_delay_seconds_,
      .referenceTime_ = date_time(incoming.reference_timestamp_seconds_)};
}

}  // namespace

api::Itinerary trip::operator()(boost::urls::url_view const& url) const {
  auto const rt = std::atomic_load(&rt_);
  auto const rtt = rt->rtt_.get();

  auto query = api::trip_params{url.params()};
  auto const api_version = get_api_version(url);

  auto const [r, _] = tags_.get_trip(tt_, rtt, query.tripId_);
  utl::verify<net::not_found_exception>(r.valid(),
                                        "trip not found: tripId={}, tt={}",
                                        query.tripId_, tt_.external_interval());

  auto const requested_fr = n::rt::frun{tt_, rtt, r};
  auto const requested_trip_id =
      tags_.id(tt_, requested_fr[0U], n::event_type::kDep);
  auto fr = requested_fr;
  fr.stop_range_.to_ = fr.size();
  fr.stop_range_.from_ = 0U;
  auto const from_l = fr[0];
  auto const to_l = fr[fr.size() - 1U];
  auto const start_time = from_l.time(n::event_type::kDep);
  auto const dest_time = to_l.time(n::event_type::kArr);
  auto cache = street_routing_cache_t{};
  auto blocked = osr::bitvec<osr::node_idx_t>{};
  auto gbfs_rd = gbfs::gbfs_routing_data{};
  auto const update_interval =
      std::chrono::seconds{config_.timetable_->update_interval_};
  auto const max_age =
      std::max(std::chrono::seconds{60}, 3 * update_interval).count();
  auto const now = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  auto const freshness_cutoff =
      vehicle_matching::freshness_cutoff(now, max_age);

  auto response = journey_to_response(
      w_, l_, pl_, tt_, tags_, canonical_stop_registry_, nullptr, nullptr, rtt,
      matches_, nullptr, shapes_, gbfs_rd, ae_, tz_,
      {.legs_ = {n::routing::journey::leg{
           n::direction::kForward, from_l.get_location_idx(),
           to_l.get_location_idx(), start_time, dest_time,
           n::routing::journey::run_enter_exit{
               fr,  // NOLINT(cppcoreguidelines-slicing)
               fr.stop_range_.from_,
               static_cast<n::stop_idx_t>(fr.stop_range_.to_ - 1U)}}},
       .start_time_ = start_time,
       .dest_time_ = dest_time,
       .dest_ = to_l.get_location_idx(),
       .transfers_ = 0U},
      tt_location{from_l.get_location_idx(),
                  from_l.get_scheduled_location_idx()},
      tt_location{to_l.get_location_idx()}, cache, &blocked, false,
      osr_parameters{}, api::PedestrianProfileEnum::FOOT,
      api::ElevationCostsEnum::NONE, query.joinInterlinedLegs_, true,
      query.detailedLegs_, false, query.withScheduledSkippedStops_,
      config_.timetable_.value().max_matching_distance_, kMaxMatchingDistance,
      api_version, false, false, query.language_, nullptr);
  if (rt->vehicle_positions_ != nullptr && !response.legs_.empty()) {
    auto const leg = query.joinInterlinedLegs_
                         ? begin(response.legs_)
                         : std::ranges::find(response.legs_, requested_trip_id,
                                             &api::Leg::tripId_);
    if (leg != end(response.legs_)) {
      leg->primaryVehicle_ = vehicle_matching::primary_vehicle(
          tags_, tt_, rtt, shapes_, *rt->vehicle_positions_, requested_fr,
          freshness_cutoff, now, query.language_);
    }
  }
  if (api_version >= 6) {
    auto const prediction = [&](n::stop_idx_t const stop_idx,
                                n::event_type const event_type) {
      return selected_prediction(tt_, tags_, fr, stop_idx, event_type,
                                 rt->vehicle_prediction_diagnostics_.get());
    };
    response.startPrediction_ = prediction(0U, n::event_type::kDep);
    response.endPrediction_ = prediction(
        static_cast<n::stop_idx_t>(fr.size() - 1U), n::event_type::kArr);
    auto intermediate = std::vector<api::IntermediateStopEvent>{};
    intermediate.reserve(fr.size() > 2U ? fr.size() - 2U : 0U);
    for (auto i = n::stop_idx_t{1U}; i + 1U < fr.size(); ++i) {
      auto const stop = fr[i];
      if (!query.withScheduledSkippedStops_ &&
          !stop.get_scheduled_stop().in_allowed() &&
          !stop.get_scheduled_stop().out_allowed() && !stop.in_allowed() &&
          !stop.out_allowed()) {
        continue;
      }
      intermediate.push_back(
          {.place_ = to_place(&tt_, &tags_, canonical_stop_registry_, w_, pl_,
                              matches_, ae_, tz_, query.language_, stop),
           .arrivalPrediction_ = prediction(i, n::event_type::kArr),
           .departurePrediction_ = prediction(i, n::event_type::kDep)});
    }
    response.intermediateStopEvents_ = std::move(intermediate);
    response.incomingLegPrediction_ = incoming_leg_prediction(
        fr.t_, requested_trip_id, rt->vehicle_prediction_diagnostics_.get());
  }
  return response;
}

}  // namespace motis::ep
