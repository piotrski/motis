#include "motis/rt_update.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "boost/asio/co_spawn.hpp"
#include "boost/asio/detached.hpp"
#include "boost/asio/experimental/parallel_group.hpp"
#include "boost/asio/redirect_error.hpp"
#include "boost/asio/steady_timer.hpp"
#include "boost/beast/core/buffers_to_string.hpp"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

#include "utl/read_file.h"
#include "utl/timer.h"
#include "utl/zip.h"

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/create_rt_timetable.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/rt/gtfsrt_update.h"
#include "nigiri/rt/rt_timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/elevators/update_elevators.h"
#include "motis/gtfsrt_trip_id_rewriter.h"
#include "motis/http_req.h"
#include "motis/railviz.h"
#include "motis/rt/auser.h"
#include "motis/rt/rt_metrics.h"
#include "motis/rt/trip_progress_diagnostics.h"
#include "motis/rt/vehicle_eta_calibration.h"
#include "motis/rt/vehicle_eta_control.h"
#include "motis/rt/vehicle_matching.h"
#include "motis/rt/vehicle_observation_history.h"
#include "motis/rt/vehicle_position.h"
#include "motis/rt/vehicle_prediction_diagnostics.h"
#include "motis/rt/vehicle_prediction_effective.h"
#include "motis/rt/vehicle_prediction_overlay.h"
#include "motis/rt/vehicle_prediction_selection.h"
#include "motis/rt/vehicle_prediction_store.h"
#include "motis/tag_lookup.h"
#include "motis/timetable/time_conv.h"

namespace n = nigiri;
namespace asio = boost::asio;
namespace fs = std::filesystem;
using asio::awaitable;

namespace motis {

void copy_elevator_footpaths(n::rt_timetable const& from,
                             n::rt_timetable& to) {
  constexpr auto kWheelchairProfile = 2U;
  to.has_td_footpaths_out_[kWheelchairProfile] =
      from.has_td_footpaths_out_[kWheelchairProfile];
  to.has_td_footpaths_in_[kWheelchairProfile] =
      from.has_td_footpaths_in_[kWheelchairProfile];
  to.td_footpaths_out_[kWheelchairProfile] =
      from.td_footpaths_out_[kWheelchairProfile];
  to.td_footpaths_in_[kWheelchairProfile] =
      from.td_footpaths_in_[kWheelchairProfile];
}

fs::path vehicle_eta_runtime_directory(data const& d) {
  if (auto const* configured = std::getenv("MOTIS_VEHICLE_ETA_RUNTIME_DIR");
      configured != nullptr && *configured != '\0') {
    return configured;
  }
  return d.path_ / "runtime" / "vehicle-eta";
}

bool vehicle_eta_calibration_enabled() {
  auto const* configured = std::getenv("MOTIS_VEHICLE_ETA_CALIBRATION");
  return configured == nullptr || std::string_view{configured} != "off";
}

asio::awaitable<ptr<elevators>> update_elevators(config const& c,
                                                 data const& d,
                                                 n::rt_timetable& new_rtt) {
  utl::verify(c.has_elevators() && c.get_elevators()->url_ && c.timetable_,
              "elevator update requires settings for timetable + elevators");
  auto const res =
      co_await http_GET(boost::urls::url{*c.get_elevators()->url_},
                        c.get_elevators()->headers_.value_or(headers_t{}),
                        std::chrono::seconds{c.get_elevators()->http_timeout_});
  co_return update_elevators(c, d, get_http_body(res), new_rtt);
}

std::string get_dump_path(auto&& ep) {
  auto const normalize = [](std::string const& x) {
    auto ret = std::string{};
    ret.resize(x.size());
    for (auto [to, from] : utl::zip(ret, x)) {
      auto const c = from;
      if (('0' <= c && c <= '9') ||  //
          ('a' <= c && c <= 'z') ||  //
          ('A' <= c && c <= 'Z')) {
        to = c;
      } else {
        to = '_';
      }
    }
    return ret;
  };
  return fmt::format("dump_rt/{}-{}", ep.tag_, normalize(ep.ep_.url_));
}

struct gtfs_rt_endpoint {
  struct last_good {
    transit_realtime::FeedMessage snapshot_;
    bool has_snapshot_{false};
    std::chrono::steady_clock::time_point received_at_{};
    std::chrono::steady_clock::time_point expires_at_{};
    std::chrono::seconds age_at_receipt_{0};
    std::int64_t reference_at_seconds_{};
    bool failed_{false};
    bool expired_{false};
  };

  config::timetable::dataset::rt ep_;
  n::source_idx_t src_;
  std::string tag_;
  gtfsrt_metrics metrics_;
  std::shared_ptr<last_good> last_good_{std::make_shared<last_good>()};
};

struct auser_endpoint {
  config::timetable::dataset::rt ep_;
  n::source_idx_t src_;
  std::string tag_;
  vdvaus_metrics metrics_;
};

std::string vehicle_feed_id(gtfs_rt_endpoint const& ep) {
  return fmt::format("{}:{}", ep.tag_, ep.ep_.hash());
}

vehicle_observation to_observation(
    vehicle_positions::vehicle_position const& position) {
  return vehicle_observation{
      .feed_id_ = position.feed_id_,
      .entity_id_ = position.entity_id_,
      .vehicle_id_ = position.vehicle_.id_,
      .trip_ = {.trip_id_ = position.trip_.trip_id_,
                .start_date_ = position.trip_.start_date_,
                .start_time_ = position.trip_.start_time_},
      .latitude_ = position.reported_position_.pos_.lat_,
      .longitude_ = position.reported_position_.pos_.lng_,
      .bearing_ = position.reported_position_.bearing_,
      .speed_mps_ = position.reported_position_.speed_mps_,
      .current_stop_sequence_ = position.current_stop_sequence_,
      .stop_id_ = position.stop_id_,
      .current_status_ = position.current_status_,
      .reported_time_ = position.reported_time_,
      .ingested_time_ = position.ingested_time_};
}

enum struct gtfsrt_payload_error { empty_body, decode_error, missing_header };

struct gtfsrt_payload_exception final : std::runtime_error {
  gtfsrt_payload_exception(gtfsrt_payload_error const error,
                           char const* const message)
      : std::runtime_error{message}, error_{error} {}

  gtfsrt_payload_error error_;
};

transit_realtime::FeedMessage validate_gtfsrt_payload(
    std::string_view const body) {
  if (body.empty()) {
    throw gtfsrt_payload_exception{gtfsrt_payload_error::empty_body,
                                   "empty GTFS-RT feed"};
  }
  auto msg = transit_realtime::FeedMessage{};
  if (!msg.ParsePartialFromArray(body.data(), static_cast<int>(body.size()))) {
    throw gtfsrt_payload_exception{gtfsrt_payload_error::decode_error,
                                   "unable to parse GTFS-RT feed"};
  }
  if (!msg.has_header()) {
    throw gtfsrt_payload_exception{gtfsrt_payload_error::missing_header,
                                   "GTFS-RT feed has no header"};
  }
  if (!msg.IsInitialized()) {
    throw gtfsrt_payload_exception{gtfsrt_payload_error::decode_error,
                                   "unable to parse GTFS-RT feed"};
  }
  return msg;
}

asio::awaitable<std::string> fetch_valid_gtfsrt_payload(
    gtfs_rt_endpoint const& endpoint, std::chrono::seconds const timeout) {
  constexpr auto kInitialAttempts = 3U;
  constexpr auto kRetryDelay = std::chrono::milliseconds{100};
  auto const attempts =
      endpoint.last_good_->has_snapshot_ ? 1U : kInitialAttempts;
  auto executor = co_await asio::this_coro::executor;
  auto timer = asio::steady_timer{executor};
  for (auto attempt = 0U; attempt != attempts; ++attempt) {
    auto error = std::exception_ptr{};
    try {
      auto const response = co_await http_GET(
          boost::urls::url{endpoint.ep_.url_},
          endpoint.ep_.headers_.value_or(headers_t{}), timeout);
      auto body = get_http_body(response);
      static_cast<void>(validate_gtfsrt_payload(body));
      co_return body;
    } catch (...) {
      error = std::current_exception();
    }
    if (attempt + 1U == attempts) {
      std::rethrow_exception(error);
    }
    timer.expires_after(kRetryDelay);
    co_await timer.async_wait(asio::use_awaitable);
  }
  std::unreachable();
}

void count_payload_error(gtfsrt_metrics const& metrics,
                         gtfsrt_payload_error const error) {
  switch (error) {
    case gtfsrt_payload_error::empty_body:
      metrics.empty_body_.Increment();
      break;
    case gtfsrt_payload_error::decode_error:
      metrics.decode_error_.Increment();
      break;
    case gtfsrt_payload_error::missing_header:
      metrics.missing_header_.Increment();
      break;
  }
}

transit_realtime::FeedMessage materialize_gtfsrt_snapshot(
    transit_realtime::FeedMessage const& base,
    transit_realtime::FeedMessage const& update) {
  auto entities = std::map<std::string, transit_realtime::FeedEntity>{};
  for (auto const& entity : base.entity()) {
    auto retained = entity;
    if (retained.has_trip_update() &&
        !retained.trip_update().has_timestamp() &&
        base.header().has_timestamp()) {
      retained.mutable_trip_update()->set_timestamp(base.header().timestamp());
    }
    entities.insert_or_assign(retained.id(), std::move(retained));
  }
  for (auto const& entity : update.entity()) {
    if (entity.has_is_deleted() && entity.is_deleted()) {
      entities.erase(entity.id());
    } else {
      entities.insert_or_assign(entity.id(), entity);
    }
  }

  auto materialized = update;
  materialized.clear_entity();
  for (auto const& [_, entity] : entities) {
    *materialized.add_entity() = entity;
  }
  materialized.mutable_header()->set_incrementality(
      transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
  return materialized;
}

std::chrono::seconds gtfsrt_payload_age(
    transit_realtime::FeedMessage const& msg,
    std::chrono::system_clock::time_point const now) {
  if (!msg.header().has_timestamp() || msg.header().timestamp() == 0U) {
    return std::chrono::seconds{0};
  }
  auto const now_seconds =
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
          .count();
  auto const timestamp = msg.header().timestamp();
  if (now_seconds <= 0 ||
      timestamp >= static_cast<std::uint64_t>(now_seconds)) {
    return std::chrono::seconds{0};
  }
  return std::chrono::seconds{now_seconds -
                              static_cast<std::int64_t>(timestamp)};
}

prediction_candidate_diagnostic provider_diagnostic(
    vehicle_stop_prediction const&,
    std::optional<std::int64_t> reference_timestamp);

struct selected_cycle_prediction {
  vehicle_prediction_cycle_result const* candidate_{};
  vehicle_prediction_selection selection_;
};

std::vector<selected_cycle_prediction> select_cycle_predictions(
    config const& c,
    std::span<vehicle_prediction_cycle_result const> const candidates,
    std::int64_t const now_seconds,
    vehicle_eta_runtime_control const& runtime_control,
    effective_vehicle_prediction_selection_state& selection_state) {
  auto selected = std::vector<selected_cycle_prediction>{};
  selected.reserve(candidates.size());
  auto const& eta = *c.timetable_->vehicle_eta_;
  for (auto const& candidate : candidates) {
    auto const has_gps = candidate.batch_.eligible();
    auto const has_provider = !candidate.provider_predictions_.empty();
    if (!candidate.mode_.has_value() ||
        candidate.batch_.transport_ == nigiri::transport::invalid() ||
        (!has_gps && !has_provider)) {
      if (candidate.mode_.has_value() &&
          candidate.batch_.transport_ != nigiri::transport::invalid()) {
        selection_state.erase(candidate.batch_.transport_,
                              candidate.trip_stop_range_);
      }
      continue;
    }
    auto const& batch = candidate.batch_;
    auto input = vehicle_prediction_selection_input{
        .transport_ = batch.transport_,
        .now_seconds_ = now_seconds};
    if (!candidate.provider_predictions_.empty()) {
      input.provider_ = timing_source_candidate{
          .source_ = vehicle_prediction_source::kProvider,
          .reference_timestamp_seconds_ =
              candidate.provider_reference_timestamp_seconds_.value_or(
                  now_seconds),
          .confidence_ = 1.0,
          .physically_reachable_ = true,
          .predictions_ = candidate.provider_predictions_};
    }
    if (has_gps) {
      input.gps_ = timing_source_candidate{
          .source_ = vehicle_prediction_source::kGps,
          .reference_timestamp_seconds_ =
              batch.candidate_reference_timestamp_seconds_.value_or(
                  now_seconds),
          .confidence_ = batch.confidence_->score_,
          .physically_reachable_ = true,
          .implied_progress_m_ = batch.implied_progress_m_,
          .predictions_ = batch.predictions_};
    }
    auto const effective =
        runtime_control.resolve(
            candidate.feed_, *candidate.mode_,
            c.vehicle_eta_mode(candidate.feed_, *candidate.mode_)) ==
        config::timetable::vehicle_eta::mode::effective;
    auto selection = vehicle_prediction_selection{};
    if (effective) {
      selection = selection_state.select(
          input, {.max_gps_age_seconds_ = eta.history_.max_age_seconds_,
                  .provider_timestamp_tolerance_seconds_ =
                      eta.selection_.provider_timestamp_tolerance_seconds_,
                  .min_gps_confidence_ = eta.selection_.min_gps_confidence_,
                  .min_selected_gps_confidence_ =
                      eta.selection_.min_selected_gps_confidence_},
          candidate.trip_stop_range_);
    } else {
      selection_state.erase(input.transport_, candidate.trip_stop_range_);
      if (input.provider_.has_value()) {
        selection = {
            .source_ = vehicle_prediction_source::kProvider,
            .reason_ = vehicle_prediction_selection_reason::kProviderOnly,
            .predictions_ = input.provider_->predictions_};
      }
    }
    selected.push_back(
        {.candidate_ = &candidate, .selection_ = std::move(selection)});
  }
  return selected;
}

std::unique_ptr<vehicle_prediction_diagnostics_store>
build_vehicle_prediction_diagnostics(
    config const& c,
    vehicle_eta_runtime_control const& runtime_control,
    std::span<vehicle_prediction_cycle_result const> const candidates,
    std::span<selected_cycle_prediction const> const selections,
    std::span<nigiri::transport const> const applied_transports,
    vehicle_prediction_overlay_state const& overlay_state,
    std::int64_t const now_seconds) {
  if (!runtime_control.enabled(c)) {
    return nullptr;
  }
  auto entries = std::vector<vehicle_prediction_diagnostic_entry>{};
  for (auto const& candidate : candidates) {
    auto const& batch = candidate.batch_;
    if (!candidate.mode_.has_value() ||
        candidate.provider_predictions_.empty() || batch.eligible() ||
        batch.transport_ == nigiri::transport::invalid() ||
        candidate.trip_id_.empty()) {
      continue;
    }
    for (auto const& provider : candidate.provider_predictions_) {
      auto const provider_value = provider_diagnostic(
          provider, candidate.provider_reference_timestamp_seconds_);
      entries.push_back(
          {.transport_ = batch.transport_,
           .static_stop_sequence_ = provider.static_stop_sequence_,
           .event_type_ = provider.event_type_,
           .trip_id_ = candidate.trip_id_,
           .trip_stop_range_ = candidate.trip_stop_range_,
           .observed_at_seconds_ = now_seconds,
           .scheduled_timestamp_seconds_ =
               provider.scheduled_timestamp_seconds_,
           .provider_ = provider_value,
           .effective_ = provider_value,
           .selected_source_ = vehicle_prediction_source::kProvider,
           .selection_reason_ =
               vehicle_prediction_selection_reason::kProviderOnly,
           .gps_estimation_rejection_ = batch.diagnostics_.rejection_,
           .latest_vehicle_observation_timestamp_seconds_ =
               candidate.latest_vehicle_observation_timestamp_seconds_});
    }
  }
  for (auto const& candidate : candidates) {
    auto const& batch = candidate.batch_;
    if (!candidate.mode_.has_value() || batch.eligible() ||
        batch.transport_ == nigiri::transport::invalid() ||
        candidate.trip_id_.empty() ||
        !batch.diagnostics_.rejection_.has_value() ||
        !candidate.provider_predictions_.empty()) {
      continue;
    }
    auto add_rejection = [&](vehicle_prediction_stop_event const& event,
                             vehicle_prediction_event_type const event_type,
                             std::int64_t const scheduled) {
      entries.push_back(
          {.transport_ = batch.transport_,
           .static_stop_sequence_ = event.static_stop_sequence_,
           .event_type_ = event_type,
           .trip_id_ = candidate.trip_id_,
           .trip_stop_range_ = candidate.trip_stop_range_,
           .observed_at_seconds_ = now_seconds,
           .scheduled_timestamp_seconds_ = scheduled,
           .effective_ = {.source_ = vehicle_prediction_source::kSchedule,
                          .predicted_timestamp_seconds_ = scheduled,
                          .delay_seconds_ = 0},
           .gps_estimation_rejection_ = *batch.diagnostics_.rejection_,
           .latest_vehicle_observation_timestamp_seconds_ =
               candidate.latest_vehicle_observation_timestamp_seconds_});
    };
    for (auto const& event : batch.diagnostic_events_) {
      add_rejection(event, vehicle_prediction_event_type::kArrival,
                    event.scheduled_arrival_timestamp_seconds_);
      add_rejection(event, vehicle_prediction_event_type::kDeparture,
                    event.scheduled_departure_timestamp_seconds_);
    }
  }
  for (auto const& selected : selections) {
    auto const& source = *selected.candidate_;
    auto const& selection = selected.selection_;
    auto const& batch = source.batch_;
    auto const overlay_applied =
        selection.source_ != vehicle_prediction_source::kGps ||
        std::ranges::binary_search(applied_transports, batch.transport_);
    for (auto const& gps : batch.predictions_) {
      auto const provider = std::ranges::find_if(
          source.provider_predictions_, [&](vehicle_stop_prediction const& x) {
            return x.static_stop_sequence_ == gps.static_stop_sequence_ &&
                   x.event_type_ == gps.event_type_ &&
                   x.scheduled_timestamp_seconds_ ==
                       gps.scheduled_timestamp_seconds_;
          });
      auto effective = prediction_candidate_diagnostic{
          .source_ = vehicle_prediction_source::kSchedule,
          .predicted_timestamp_seconds_ = gps.scheduled_timestamp_seconds_};
      auto provider_value = std::optional<prediction_candidate_diagnostic>{};
      if (provider != end(source.provider_predictions_)) {
        provider_value = provider_diagnostic(
            *provider, source.provider_reference_timestamp_seconds_);
        effective = *provider_value;
      }
      auto const gps_diagnostic = prediction_candidate_diagnostic{
          .source_ = vehicle_prediction_source::kGps,
          .predicted_timestamp_seconds_ = gps.predicted_timestamp_seconds_,
          .delay_seconds_ = gps.delay_seconds_,
          .confidence_ = batch.confidence_->score_,
          .reference_timestamp_seconds_ =
              batch.candidate_reference_timestamp_seconds_};
      if (selection.source_ == vehicle_prediction_source::kGps &&
          overlay_applied) {
        effective = gps_diagnostic;
        auto const rendered = std::ranges::find_if(
            overlay_state.rendered_events_, [&](auto const& event) {
              return event.transport_ == batch.transport_ &&
                     event.trip_stop_range_ == source.trip_stop_range_ &&
                     event.static_stop_sequence_ == gps.static_stop_sequence_ &&
                     event.event_type_ == gps.event_type_ &&
                     event.effective_timestamp_seconds_.has_value();
            });
        if (rendered != end(overlay_state.rendered_events_)) {
          effective.predicted_timestamp_seconds_ =
              *rendered->effective_timestamp_seconds_;
          effective.delay_seconds_ = effective.predicted_timestamp_seconds_ -
                                     gps.scheduled_timestamp_seconds_;
        }
      }
      auto const selected_source =
          overlay_applied ? selection.source_
                          : (provider_value.has_value()
                                 ? vehicle_prediction_source::kProvider
                                 : vehicle_prediction_source::kSchedule);
      entries.push_back(
          {.transport_ = batch.transport_,
           .static_stop_sequence_ = gps.static_stop_sequence_,
           .event_type_ = gps.event_type_,
           .trip_id_ = source.trip_id_,
           .trip_stop_range_ = source.trip_stop_range_,
           .observed_at_seconds_ = now_seconds,
           .scheduled_timestamp_seconds_ = gps.scheduled_timestamp_seconds_,
           .provider_ = provider_value,
           .gps_ = gps_diagnostic,
           .effective_ = effective,
           .selected_source_ = selected_source,
           .selection_reason_ =
               overlay_applied
                   ? selection.reason_
                   : (provider_value.has_value()
                          ? vehicle_prediction_selection_reason::kProviderOnly
                          : vehicle_prediction_selection_reason::
                                kNoUsableCandidate),
           .provider_rejection_ = selection.diagnostics_.provider_rejection_,
           .gps_rejection_ = selection.diagnostics_.gps_rejection_,
           .latest_vehicle_observation_timestamp_seconds_ =
               source.latest_vehicle_observation_timestamp_seconds_,
           .gps_context_ = source.context_,
           .context_ = selected_source == vehicle_prediction_source::kGps
                           ? source.context_
                           : vehicle_prediction_context::kDirect,
           .incoming_leg_provenance_ = source.incoming_leg_provenance_,
           .selected_confidence_ = selection.diagnostics_.selected_confidence_,
           .candidate_timestamp_skew_seconds_ =
               selection.diagnostics_.candidate_timestamp_skew_seconds_,
           .projection_error_m_ = batch.confidence_->lateral_error_m_,
           .progress_difference_m_ =
               selection.diagnostics_.progress_difference_m_,
           .source_transition_ = selection.diagnostics_.source_transition_,
           .provider_recovery_ = selection.diagnostics_.provider_recovery_,
           .flap_ = selection.diagnostics_.flap_,
           .provider_consistent_cycles_ =
               selection.diagnostics_.provider_consistent_cycles_});
    }
  }
  auto const max_age = c.timetable_->vehicle_eta_->history_.max_age_seconds_;
  return vehicle_prediction_diagnostics_store::build(
      true, std::move(entries), now_seconds,
      {.max_age_seconds_ = max_age, .max_entries_ = 100'000U});
}

prediction_candidate_diagnostic provider_diagnostic(
    vehicle_stop_prediction const& prediction,
    std::optional<std::int64_t> const reference_timestamp) {
  return {
      .source_ = vehicle_prediction_source::kProvider,
      .predicted_timestamp_seconds_ = prediction.predicted_timestamp_seconds_,
      .delay_seconds_ = prediction.delay_seconds_,
      .confidence_ = 1.0,
      .reference_timestamp_seconds_ = reference_timestamp};
}

std::optional<resolved_provider_trip> resolve_provider_trip(
    data const& d,
    date::sys_days const today,
    gtfs_rt_endpoint const& endpoint,
    n::rt_timetable const& staged,
    transit_realtime::TripDescriptor const& descriptor) {
  auto const [run, trip] = n::rt::gtfsrt_resolve_run(today, *d.tt_, &staged,
                                                     endpoint.src_, descriptor);
  if (!run.valid() || !run.is_scheduled() || trip == n::trip_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const fr = n::rt::frun{*d.tt_, &staged, run};
  auto const encoded_sequences = d.tt_->trip_stop_seq_numbers_[trip];
  auto const sequence_number_range = n::loader::gtfs::stop_seq_number_range{
      {encoded_sequences.data(), encoded_sequences.size()},
      static_cast<n::stop_idx_t>(run.stop_range_.size())};
  auto const sequence_numbers = std::vector<n::stop_idx_t>{
      begin(sequence_number_range), end(sequence_number_range)};
  auto const trip_stops = std::vector<n::rt::run_stop>{begin(fr), end(fr)};
  auto stops = std::vector<scheduled_provider_stop>{};
  stops.reserve(trip_stops.size());
  for (auto const [stop, sequence] : utl::zip(trip_stops, sequence_numbers)) {
    auto const arrival = stop.stop_idx_ == fr.stop_range_.from_
                             ? n::event_type::kDep
                             : n::event_type::kArr;
    auto const departure = stop.stop_idx_ + 1U == fr.stop_range_.to_
                               ? n::event_type::kArr
                               : n::event_type::kDep;
    stops.push_back(
        {.static_stop_sequence_ = static_cast<unsigned>(sequence),
         .stop_id_ =
             std::string{
                 d.tt_->locations_.ids_[stop.get_location_idx()].view()},
         .arrival_timestamp_seconds_ = to_seconds(stop.scheduled_time(arrival)),
         .departure_timestamp_seconds_ =
             to_seconds(stop.scheduled_time(departure)),
         .effective_arrival_timestamp_seconds_ = to_seconds(stop.time(arrival)),
         .effective_departure_timestamp_seconds_ =
             to_seconds(stop.time(departure))});
  }
  return resolved_provider_trip{
      .transport_ = run.t_,
      .trip_stop_range_ = run.stop_range_,
      .stops_ = std::move(stops),
      .trip_id_ = d.tags_->id(*d.tt_, fr[0], n::event_type::kDep),
      .mode_ = fr[0].get_clasz(n::event_type::kDep)};
}

std::vector<vehicle_prediction_cycle_result> provider_prediction_candidates(
    config const& c,
    vehicle_eta_runtime_control const& runtime_control,
    data const& d,
    date::sys_days const today,
    gtfs_rt_endpoint const& endpoint,
    transit_realtime::FeedMessage const& message,
    n::rt_timetable const& staged,
    std::int64_t const observed_at) {
  if (!runtime_control.enabled(c)) {
    return {};
  }
  auto const extracted = extract_provider_timing(
      message, [&](transit_realtime::TripDescriptor const& descriptor) {
        return resolve_provider_trip(d, today, endpoint, staged, descriptor);
      });
  auto results = std::vector<vehicle_prediction_cycle_result>{};
  for (auto const& provider : extracted.candidates_) {
    if (!provider.mode_.has_value() ||
        runtime_control.resolve(c, endpoint.tag_, *provider.mode_) ==
            config::timetable::vehicle_eta::mode::off) {
      continue;
    }
    auto result = vehicle_prediction_cycle_result{
        .feed_ = endpoint.tag_,
        .trip_id_ = provider.trip_id_,
        .mode_ = provider.mode_,
        .trip_stop_range_ = provider.trip_stop_range_};
    result.batch_.transport_ = provider.transport_;
    result.provider_reference_timestamp_seconds_ =
        provider.trip_update_timestamp_seconds_
            .or_else([&] { return provider.feed_timestamp_seconds_; })
            .value_or(observed_at);
    auto const add_event = [&](unsigned const sequence,
                               vehicle_prediction_event_type const event_type,
                               std::optional<std::int64_t> const scheduled,
                               std::optional<std::int64_t> const predicted) {
      if (!scheduled.has_value() || !predicted.has_value()) {
        return;
      }
      auto const difference = [](std::int64_t const left,
                                 std::int64_t const right)
          -> std::optional<std::int64_t> {
        if ((right > 0 &&
             left < std::numeric_limits<std::int64_t>::min() + right) ||
            (right < 0 &&
             left > std::numeric_limits<std::int64_t>::max() + right)) {
          return std::nullopt;
        }
        return left - right;
      };
      auto const delay = difference(*predicted, *scheduled);
      auto const horizon = difference(*predicted, observed_at);
      if (!delay.has_value() || !horizon.has_value()) {
        return;
      }
      result.provider_predictions_.push_back(
          {.static_stop_sequence_ = sequence,
           .event_type_ = event_type,
           .scheduled_timestamp_seconds_ = *scheduled,
           .predicted_timestamp_seconds_ = *predicted,
           .delay_seconds_ = *delay,
           .horizon_seconds_ = *horizon});
    };
    for (auto const& stop : provider.stops_) {
      auto const resolved = std::ranges::find(
          provider.resolved_stops_, stop.static_stop_sequence_,
          &scheduled_provider_stop::static_stop_sequence_);
      if (resolved == end(provider.resolved_stops_)) {
        continue;
      }
      add_event(stop.static_stop_sequence_,
                vehicle_prediction_event_type::kArrival,
                resolved->arrival_timestamp_seconds_,
                stop.arrival_timestamp_seconds_);
      add_event(stop.static_stop_sequence_,
                vehicle_prediction_event_type::kDeparture,
                resolved->departure_timestamp_seconds_,
                stop.departure_timestamp_seconds_);
    }
    if (!result.provider_predictions_.empty()) {
      results.emplace_back(std::move(result));
    }
  }
  return results;
}

void run_rt_update(boost::asio::io_context& ioc,
                   config const& c,
                   data& d,
                   rt_update_hooks hooks) {
  boost::asio::co_spawn(
      ioc,
      [&c, &d, hooks = std::move(hooks)]() -> awaitable<void> {
        auto const dump_rt = fs::is_directory("dump_rt");
        if (dump_rt) {
          fmt::println("WARNING: DUMPING TO dump_rt\n");
        }

        auto executor = co_await asio::this_coro::executor;
        auto timer = asio::steady_timer{executor};
        auto ec = boost::system::error_code{};
        auto const metric_families = rt_metric_families{d.metrics_->registry_};
        auto& cycle_completed = metric_families.rt_cycle_completed_.Add({});
        auto& cycle_duration =
            metric_families.rt_cycle_duration_seconds_.Add({});
        auto& cycle_overrun = metric_families.rt_cycle_overrun_.Add({});
        constexpr auto kPhases = std::array{"initialization",
                                            "feed_wait",
                                            "feed_prepare",
                                            "feed_apply",
                                            "history",
                                            "progress_projection",
                                            "gps_estimation",
                                            "continuation_provider_merge",
                                            "calibration_capture",
                                            "metrics_aggregation",
                                            "diagnostics_materialization",
                                            "selection",
                                            "timetable_clone",
                                            "overlay",
                                            "lower_bounds",
                                            "railviz",
                                            "elevators",
                                            "publication"};
        auto phase_metrics = std::map<std::string_view, prometheus::Gauge*>{};
        auto phase_values = std::map<std::string_view, double>{};
        for (auto const phase : kPhases) {
          phase_metrics.emplace(phase,
                                &metric_families.rt_cycle_phase_seconds_.Add(
                                    {{"phase", std::string{phase}}}));
          phase_values.emplace(phase, 0.0);
        }
        constexpr auto kWorkloads = std::array{"endpoints",
                                               "fetched_bytes",
                                               "vehicles",
                                               "history_observations",
                                               "progress_diagnostics",
                                               "provider_candidates",
                                               "candidates",
                                               "predicted_events",
                                               "selected_overlays",
                                               "calibration_enabled",
                                               "calibration_pending",
                                               "calibration_completed",
                                               "calibration_appended_records",
                                               "calibration_appended_bytes"};
        auto workload_metrics =
            std::map<std::string_view, prometheus::Gauge*>{};
        auto workload_values = std::map<std::string_view, std::size_t>{};
        for (auto const workload : kWorkloads) {
          workload_metrics.emplace(workload,
                                   &metric_families.rt_cycle_workload_.Add(
                                       {{"kind", std::string{workload}}}));
          workload_values.emplace(workload, 0U);
        }
        auto const set_phase =
            [&](std::string_view const phase,
                std::chrono::steady_clock::time_point const started) {
              phase_values.at(phase) =
                  std::chrono::duration<double>{
                      std::chrono::steady_clock::now() - started}
                      .count();
            };
        auto const set_workload = [&](std::string_view const workload,
                                      std::size_t const value) {
          workload_values.at(workload) = value;
        };
        auto& history_active_vehicles =
            metric_families.vehicle_eta_history_active_vehicles_.Add({});
        auto& history_observations =
            metric_families.vehicle_eta_history_observations_.Add({});
        auto& history_memory_bytes =
            metric_families.vehicle_eta_history_memory_bytes_.Add({});
        auto& history_update_seconds =
            metric_families.vehicle_eta_history_update_seconds_.Add({});
        auto& progress_evaluation_seconds =
            metric_families.vehicle_eta_progress_evaluation_seconds_.Add({});
        auto& candidate_evaluation_seconds =
            metric_families.vehicle_eta_candidate_evaluation_seconds_.Add({});
        auto& candidate_memory_bytes =
            metric_families.vehicle_eta_candidate_memory_bytes_.Add({});
        auto progress_outcome_metrics =
            std::map<std::tuple<std::string, std::string, std::string>,
                     prometheus::Gauge*>{};
        auto progress_lateral_metrics =
            std::map<std::tuple<std::string, std::string, std::string>,
                     prometheus::Gauge*>{};
        auto candidate_outcome_metrics =
            std::map<std::tuple<std::string, std::string, std::string>,
                     prometheus::Gauge*>{};
        auto candidate_horizon_metrics =
            std::map<std::tuple<std::string, std::string, std::string>,
                     prometheus::Gauge*>{};
        auto candidate_error_metrics = std::map<
            std::tuple<std::string, std::string, std::string, std::string>,
            prometheus::Gauge*>{};
        auto prediction_overlay_state = vehicle_prediction_overlay_state{};
        auto const eta_runtime_directory = vehicle_eta_runtime_directory(d);
        auto eta_control =
            vehicle_eta_runtime_control{eta_runtime_directory / "control.json"};
        auto const calibration_enabled = vehicle_eta_calibration_enabled();
        auto eta_calibration = std::optional<vehicle_eta_calibration>{};
        if (calibration_enabled) {
          eta_calibration.emplace(eta_runtime_directory / "calibration");
        }
        static_cast<void>(eta_control.reload());

        auto const endpoints = [&]() {
          auto endpoints =
              std::vector<std::variant<gtfs_rt_endpoint, auser_endpoint>>{};
          for (auto const& [tag, dataset] : c.timetable_->datasets_) {
            if (dataset.rt_.has_value()) {
              auto const src = d.tags_->get_src(tag);
              auto gtfsrt_endpoint_idx = 0U;
              for (auto const& ep : *dataset.rt_) {
                switch (ep.protocol_) {
                  case config::timetable::dataset::rt::protocol::gtfsrt: {
                    auto const endpoint_id =
                        std::to_string(gtfsrt_endpoint_idx++);
                    endpoints.push_back(gtfs_rt_endpoint{
                        ep, src, tag,
                        gtfsrt_metrics{tag, endpoint_id, metric_families}});
                    break;
                  }
                  case config::timetable::dataset::rt::protocol::siri_json:
                  case config::timetable::dataset::rt::protocol::siri:
                    [[fallthrough]];
                  case config::timetable::dataset::rt::protocol::auser:
                    endpoints.push_back(auser_endpoint{
                        ep, src, tag, vdvaus_metrics{tag, metric_families}});
                    break;
                }
              }
            }
          }
          return endpoints;
        }();
        auto const has_gtfsrt_endpoint =
            std::any_of(endpoints.begin(), endpoints.end(), [](auto const& ep) {
              return std::holds_alternative<gtfs_rt_endpoint>(ep);
            });
        auto const has_auser_endpoint =
            std::any_of(endpoints.begin(), endpoints.end(), [](auto const& ep) {
              return std::holds_alternative<auser_endpoint>(ep);
            });
        auto const rebuild_gtfsrt_from_materialized_snapshots =
            c.timetable_->incremental_rt_update_ && has_gtfsrt_endpoint;
        auto const mixed_incremental_sources =
            rebuild_gtfsrt_from_materialized_snapshots && has_auser_endpoint;
        auto auser_rtt = std::unique_ptr<n::rt_timetable>{};
        auto auser_rtt_day = std::optional<date::sys_days>{};
        auto effective_prediction_selection_state =
            effective_vehicle_prediction_selection_state{};

        while (true) {
          // Remember when we started, so we can schedule the next update.
          auto const start = std::chrono::steady_clock::now();
          for (auto& [_, value] : phase_values) {
            value = 0.0;
          }
          for (auto& [_, value] : workload_values) {
            value = 0U;
          }
          set_workload("endpoints", endpoints.size());
          set_workload("calibration_enabled", calibration_enabled ? 1U : 0U);

          try {
            auto t = utl::scoped_timer{"rt update"};
            auto const initialization_started =
                std::chrono::steady_clock::now();
            static_cast<void>(eta_control.reload());

            // Create new real-time timetable.
            auto const now =
                hooks.now_ ? hooks.now_() : std::chrono::system_clock::now();
            auto const today = std::chrono::time_point_cast<date::days>(now);
            auto const published_rt = std::atomic_load(&d.rt_);
            auto const* published_provider_rtt =
                published_rt->provider_rtt_ != nullptr
                    ? published_rt->provider_rtt_.get()
                    : published_rt->rtt_.get();
            auto const auser_day_rollover =
                has_auser_endpoint &&
                (mixed_incremental_sources
                     ? auser_rtt_day != today
                     : published_provider_rtt->base_day_ != today);
            if (published_provider_rtt->base_day_ != today) {
              effective_prediction_selection_state.reset();
            }
            if (auser_day_rollover) {
              auto reset_urls = std::set<std::string_view>{};
              for (auto const& endpoint : endpoints) {
                if (auto const* a = std::get_if<auser_endpoint>(&endpoint);
                    a != nullptr && reset_urls.emplace(a->ep_.url_).second) {
                  d.auser_->at(a->ep_.url_).reset_for_resync();
                }
              }
            }
            if (mixed_incremental_sources && auser_rtt_day != today) {
              auser_rtt = std::make_unique<n::rt_timetable>(
                  n::rt::create_rt_timetable(*d.tt_, today));
              auser_rtt_day = today;
            }
            auto const reuse_provider_rtt =
                c.timetable_->incremental_rt_update_ &&
                !rebuild_gtfsrt_from_materialized_snapshots &&
                !auser_day_rollover;
            auto rtt = std::make_unique<n::rt_timetable>(
                reuse_provider_rtt
                    ? n::rt_timetable{*published_provider_rtt}
                    : n::rt::create_rt_timetable(*d.tt_, today));
            if (!reuse_provider_rtt && c.has_elevators()) {
              copy_elevator_footpaths(*published_rt->rtt_, *rtt);
            }

            auto history_update_cpu = std::chrono::nanoseconds{0};
            auto const history_copy_started = std::chrono::steady_clock::now();
            auto vehicle_position_store =
                std::make_unique<vehicle_positions::vehicle_position_store>(
                    published_rt->vehicle_positions_ != nullptr
                        ? *published_rt->vehicle_positions_
                        : vehicle_positions::vehicle_position_store{});
            auto vehicle_history =
                eta_control.enabled(c)
                    ? std::make_unique<vehicle_observation_history>(
                          published_rt->vehicle_observation_history_ != nullptr
                              ? *published_rt->vehicle_observation_history_
                              : vehicle_observation_history{})
                    : nullptr;
            history_update_cpu +=
                std::chrono::steady_clock::now() - history_copy_started;
            auto const history_policy =
                c.timetable_->vehicle_eta_
                    ? observation_history_policy{std::chrono::seconds{
                                                     c.timetable_->vehicle_eta_
                                                         ->history_
                                                         .retention_seconds_},
                                                 c.timetable_->vehicle_eta_
                                                     ->history_
                                                     .max_observations_per_vehicle_}
                    : observation_history_policy{std::chrono::seconds{1}, 1U};
            set_phase("initialization", initialization_started);

            // Schedule updates for each real-time endpoint.
            auto const timeout =
                std::chrono::seconds{c.timetable_->http_timeout_};

            using stats_t =
                std::variant<n::rt::statistics, n::rt::vdv_aus::statistics>;
            struct update_result {
              stats_t stats_;
              bool source_success_{true};
            };
            struct collected_update {
              std::optional<std::string> body_;
              std::exception_ptr error_;
            };
            struct prepared_gtfsrt_update {
              std::size_t endpoint_idx_;
              std::optional<transit_realtime::FeedMessage> msg_;
              bool source_success_{true};
              bool commit_last_good_{false};
              bool apply_positions_differential_{false};
              std::vector<std::string> deleted_entity_ids_{};
              std::optional<transit_realtime::FeedMessage>
                  differential_positions_msg_{};
              std::optional<std::int64_t> provider_observed_at_seconds_{};
            };
            struct prepared_auser_update {
              std::size_t endpoint_idx_;
              std::optional<std::string> body_;
            };
            using prepared_update =
                std::variant<prepared_gtfsrt_update, prepared_auser_update>;
            struct update_group {
              std::string tag_;
              n::source_idx_t src_;
              std::vector<prepared_update> updates_;
            };

            auto const cache_age =
                [](gtfs_rt_endpoint::last_good const& state,
                   std::chrono::steady_clock::time_point const now) {
                  return state.age_at_receipt_ +
                         std::chrono::duration_cast<std::chrono::seconds>(
                             now - state.received_at_);
                };
            auto const expire_cache =
                [&](gtfs_rt_endpoint const& g,
                    std::chrono::steady_clock::time_point const now) {
                  auto& state = *g.last_good_;
                  if (!state.has_snapshot_ || now < state.expires_at_) {
                    return false;
                  }
                  auto const age = cache_age(state, now);
                  if (!state.expired_) {
                    g.metrics_.last_good_expiry_.Increment();
                  }
                  state.has_snapshot_ = false;
                  state.expired_ = true;
                  g.metrics_.set_source_state(gtfsrt_source_state::expired,
                                              static_cast<double>(age.count()),
                                              false);
                  return true;
                };
            auto const commit_last_good =
                [&](gtfs_rt_endpoint const& g,
                    transit_realtime::FeedMessage candidate) {
                  auto& state = *g.last_good_;
                  auto const received_at = std::chrono::steady_clock::now();
                  auto const age_at_receipt =
                      c.timetable_->canned_rt_
                          ? std::chrono::seconds{0}
                          : gtfsrt_payload_age(
                                candidate, std::chrono::system_clock::now());
                  auto const ttl = std::chrono::seconds{g.ep_.last_good_ttl_};
                  if (state.failed_ || state.expired_) {
                    g.metrics_.recovery_.Increment();
                  }
                  state.snapshot_ = std::move(candidate);
                  state.has_snapshot_ = true;
                  state.received_at_ = received_at;
                  state.age_at_receipt_ = age_at_receipt;
                  state.reference_at_seconds_ =
                      std::chrono::duration_cast<std::chrono::seconds>(
                          now.time_since_epoch())
                          .count();
                  state.expires_at_ = received_at + ttl - age_at_receipt;
                  state.failed_ = false;
                  state.expired_ = false;
                  g.metrics_.set_source_state(
                      gtfsrt_source_state::live,
                      static_cast<double>(age_at_receipt.count()), true);
                };
            auto const prepare_last_good = [&](std::size_t const endpoint_idx,
                                               gtfs_rt_endpoint const& g) {
              auto const now = std::chrono::steady_clock::now();
              expire_cache(g, now);
              if (g.last_good_->has_snapshot_) {
                g.last_good_->failed_ = true;
                g.metrics_.last_good_reuse_.Increment();
                g.metrics_.set_source_state(
                    gtfsrt_source_state::replay,
                    static_cast<double>(cache_age(*g.last_good_, now).count()),
                    true);
                return prepared_gtfsrt_update{
                    .endpoint_idx_ = endpoint_idx,
                    .msg_ = g.last_good_->snapshot_,
                    .source_success_ = false,
                    .provider_observed_at_seconds_ =
                        g.last_good_->reference_at_seconds_};
              }
              g.last_good_->failed_ = true;
              auto const expired = g.last_good_->expired_;
              auto const age = expired
                                   ? static_cast<double>(
                                         cache_age(*g.last_good_, now).count())
                                   : 0.0;
              g.metrics_.set_source_state(expired
                                              ? gtfsrt_source_state::expired
                                              : gtfsrt_source_state::no_base,
                                          age, false);
              return prepared_gtfsrt_update{endpoint_idx, std::nullopt, false,
                                            false};
            };
            auto const prepare_valid_gtfsrt =
                [&](std::size_t const endpoint_idx, gtfs_rt_endpoint const& g,
                    std::string_view const body) {
                  auto msg = validate_gtfsrt_payload(body);
                  auto const differential =
                      msg.header().incrementality() ==
                      transit_realtime::FeedHeader_Incrementality_DIFFERENTIAL;
                  expire_cache(g, std::chrono::steady_clock::now());
                  auto const apply_positions_differential = differential;
                  auto deleted_entity_ids = std::vector<std::string>{};
                  if (apply_positions_differential) {
                    for (auto const& entity : msg.entity()) {
                      if (entity.has_is_deleted() && entity.is_deleted()) {
                        deleted_entity_ids.emplace_back(entity.id());
                      }
                    }
                  }
                  auto candidate = differential
                                       ? materialize_gtfsrt_snapshot(
                                             g.last_good_->snapshot_, msg)
                                       : std::move(msg);
                  candidate.mutable_header()->set_incrementality(
                      transit_realtime::FeedHeader_Incrementality_FULL_DATASET);
                  auto const age_at_receipt = gtfsrt_payload_age(
                      candidate, std::chrono::system_clock::now());
                  if (!c.timetable_->canned_rt_ &&
                      age_at_receipt >=
                          std::chrono::seconds{g.ep_.last_good_ttl_}) {
                    g.metrics_.updates_error_.Increment();
                    auto& state = *g.last_good_;
                    state.failed_ = true;
                    if (state.has_snapshot_) {
                      return prepare_last_good(endpoint_idx, g);
                    }
                    if (state.expired_) {
                      g.metrics_.set_source_state(
                          gtfsrt_source_state::expired,
                          static_cast<double>(
                              cache_age(state, std::chrono::steady_clock::now())
                                  .count()),
                          false);
                      return prepared_gtfsrt_update{endpoint_idx, std::nullopt,
                                                    false, false};
                    }
                    g.metrics_.last_good_expiry_.Increment();
                    state.snapshot_.Clear();
                    state.has_snapshot_ = false;
                    state.received_at_ = std::chrono::steady_clock::now();
                    state.expires_at_ = state.received_at_;
                    state.age_at_receipt_ = age_at_receipt;
                    state.expired_ = true;
                    g.metrics_.set_source_state(
                        gtfsrt_source_state::expired,
                        static_cast<double>(age_at_receipt.count()), false);
                    return prepared_gtfsrt_update{endpoint_idx, std::nullopt,
                                                  false, false};
                  }
                  return prepared_gtfsrt_update{
                      endpoint_idx,
                      std::move(candidate),
                      true,
                      true,
                      apply_positions_differential,
                      std::move(deleted_entity_ids),
                      differential ? std::make_optional(std::move(msg))
                                   : std::nullopt};
                };
            auto provider_candidates =
                std::vector<vehicle_prediction_cycle_result>{};
            auto const append_provider_candidates =
                [&](gtfs_rt_endpoint const& g,
                    transit_realtime::FeedMessage const& msg,
                    n::rt_timetable const& staged,
                    std::optional<std::int64_t> const reference_at) {
                  auto const observed_at = reference_at.value_or(
                      std::chrono::duration_cast<std::chrono::seconds>(
                          now.time_since_epoch())
                          .count());
                  auto extracted = provider_prediction_candidates(
                      c, eta_control, d, today, g, msg, staged, observed_at);
                  provider_candidates.insert(
                      end(provider_candidates),
                      std::make_move_iterator(begin(extracted)),
                      std::make_move_iterator(end(extracted)));
                };
            auto const apply_gtfsrt = [&](gtfs_rt_endpoint const& g,
                                          prepared_gtfsrt_update const&
                                              prepared,
                                          transit_realtime::FeedMessage const&
                                              msg,
                                          bool const fallback) {
              // GTFS-RT application mutates incrementally and can throw.
              // Apply to a private copy so a failed primary or fallback
              // cannot leak a partially changed timetable into this cycle.
              auto staged = *rtt;
              auto const has_trip_updates = std::ranges::any_of(
                  msg.entity(), &transit_realtime::FeedEntity::has_trip_update);
              auto const has_vehicle_positions = std::ranges::any_of(
                  msg.entity(), &transit_realtime::FeedEntity::has_vehicle);
              auto stats =
                  has_trip_updates || !has_vehicle_positions
                      ? n::rt::gtfsrt_update_msg(*d.tt_, staged, g.src_, g.tag_,
                                                 msg)
                      : n::rt::statistics{
                            .total_entities_ = msg.entity_size(),
                            .total_entities_success_ = msg.entity_size(),
                            .total_vehicles_ = static_cast<
                                int>(std::ranges::count_if(
                                msg.entity(),
                                &transit_realtime::FeedEntity::has_vehicle)),
                            .feed_timestamp_ =
                                msg.has_header() && msg.header().has_timestamp()
                                    ? date::sys_seconds{std::chrono::seconds{
                                          msg.header().timestamp()}}
                                    : date::sys_seconds{}};
              if ((has_trip_updates || !has_vehicle_positions) &&
                  hooks.after_gtfsrt_apply_) {
                hooks.after_gtfsrt_apply_(prepared.endpoint_idx_, fallback);
              }
              if (has_trip_updates) {
                append_provider_candidates(
                    g, msg, staged, prepared.provider_observed_at_seconds_);
              }

              if (prepared.source_success_ && !fallback) {
                auto feed_id = vehicle_feed_id(g);
                auto const& positions_msg =
                    prepared.differential_positions_msg_.has_value()
                        ? *prepared.differential_positions_msg_
                        : msg;
                auto positions =
                    vehicle_positions::parse_gtfsrt_vehicle_positions(
                        feed_id, positions_msg,
                        std::chrono::duration_cast<std::chrono::seconds>(
                            now.time_since_epoch())
                            .count());
                if (c.timetable_->canned_rt_) {
                  for (auto& position : positions) {
                    position.reported_time_ = position.ingested_time_;
                  }
                }
                if (vehicle_history != nullptr) {
                  auto const history_replace_started =
                      std::chrono::steady_clock::now();
                  auto observations =
                      utl::to_vec(positions, [](auto const& position) {
                        return to_observation(position);
                      });
                  auto const ingested_at =
                      std::chrono::duration_cast<std::chrono::seconds>(
                          now.time_since_epoch())
                          .count();
                  if (prepared.apply_positions_differential_) {
                    vehicle_history->update_feed(feed_id, observations,
                                                 prepared.deleted_entity_ids_,
                                                 ingested_at, history_policy);
                  } else {
                    vehicle_history->replace_feed(feed_id, observations,
                                                  ingested_at, history_policy);
                  }
                  history_update_cpu += std::chrono::steady_clock::now() -
                                        history_replace_started;
                }
                if (prepared.apply_positions_differential_) {
                  vehicle_position_store->update_feed(
                      std::move(feed_id), std::move(positions),
                      prepared.deleted_entity_ids_);
                } else {
                  vehicle_position_store->replace_feed(std::move(feed_id),
                                                       std::move(positions));
                }
              }
              if (prepared.commit_last_good_) {
                commit_last_good(g, msg);
              }
              rtt = std::make_unique<n::rt_timetable>(std::move(staged));
              return stats;
            };

            // Collect every response before parsing or mutating the timetable.
            auto const feed_wait_started = std::chrono::steady_clock::now();
            auto collected = std::vector<collected_update>{};
            collected.reserve(endpoints.size());
            if (c.timetable_->canned_rt_) {
              fmt::println("WARNING: READING CANNED RT");
              for (auto const& ep : endpoints) {
                auto const path = std::visit(
                    [](auto const& x) { return get_dump_path(x); }, ep);
                collected.push_back({.body_ = utl::read_file(path.c_str())});
              }
            } else if (!endpoints.empty()) {
              auto awaitables = utl::to_vec(
                  endpoints,
                  [&](std::variant<gtfs_rt_endpoint, auser_endpoint> const& x) {
                    return boost::asio::co_spawn(
                        executor,
                        [&, endpoint = &x]() -> awaitable<std::string> {
                          co_return co_await std::visit(
                              utl::overloaded{
                                  [&](gtfs_rt_endpoint const& g)
                                      -> awaitable<std::string> {
                                    g.metrics_.updates_requested_.Increment();
                                    auto body =
                                        co_await fetch_valid_gtfsrt_payload(
                                            g, timeout);
                                    if (g.tag_ == "pl-Warszawa") {
                                      auto feed =
                                          transit_realtime::FeedMessage{};
                                      if (feed.ParseFromString(body)) {
                                        auto const lookup =
                                            build_gtfsrt_trip_id_lookup(
                                                *d.tt_, *d.tags_, *rtt, g.tag_);
                                        if (rewrite_gtfsrt_trip_ids(feed,
                                                                    lookup)) {
                                          body = feed.SerializeAsString();
                                        }
                                      }
                                    }
                                    if (dump_rt) {
                                      std::ofstream{get_dump_path(g),
                                                    std::ios::binary}
                                          .write(body.data(), static_cast<long>(
                                                                  body.size()));
                                    }
                                    co_return body;
                                  },
                                  [&](auser_endpoint const& a)
                                      -> awaitable<std::string> {
                                    a.metrics_.updates_requested_.Increment();
                                    auto& auser = d.auser_->at(a.ep_.url_);
                                    auto const fetch_url = boost::urls::url{
                                        auser.fetch_url(a.ep_.url_)};
                                    fmt::println("[auser] fetch url: {}",
                                                 fetch_url.c_str());
                                    auto const res = co_await http_GET(
                                        fetch_url,
                                        a.ep_.headers_.value_or(headers_t{}),
                                        timeout);
                                    auto body = get_http_body(res);
                                    if (dump_rt) {
                                      std::ofstream{get_dump_path(a),
                                                    std::ios::binary}
                                          .write(body.data(), static_cast<long>(
                                                                  body.size()));
                                    }
                                    co_return body;
                                  }},
                              *endpoint);
                        },
                        asio::deferred);
                  });

              auto [_, exceptions, bodies] =
                  co_await asio::experimental::make_parallel_group(awaitables)
                      .async_wait(asio::experimental::wait_for_all(),
                                  asio::use_awaitable);
              for (auto&& [ex, body] : utl::zip(exceptions, bodies)) {
                if (ex) {
                  collected.push_back({std::nullopt, ex});
                } else {
                  collected.push_back({std::move(body), {}});
                }
              }
            }
            set_phase("feed_wait", feed_wait_started);
            set_workload("fetched_bytes",
                         std::accumulate(
                             begin(collected), end(collected), std::size_t{0U},
                             [](std::size_t const total,
                                collected_update const& update) {
                               return total + (update.body_.has_value()
                                                   ? update.body_->size()
                                                   : 0U);
                             }));

            auto const feed_prepare_started = std::chrono::steady_clock::now();
            auto results = std::vector<update_result>(endpoints.size());
            auto groups = std::vector<update_group>{};
            auto const add_to_group = [&](std::size_t const endpoint_idx,
                                          prepared_update update) {
              auto const [tag, src] = std::visit(
                  [](auto const& ep) { return std::pair{ep.tag_, ep.src_}; },
                  endpoints[endpoint_idx]);
              auto const it = std::find_if(
                  groups.begin(), groups.end(), [&](update_group const& group) {
                    return group.tag_ == tag && group.src_ == src;
                  });
              auto& group =
                  it == groups.end() ? groups.emplace_back(tag, src) : *it;
              group.updates_.push_back(std::move(update));
            };

            // Parse and group in configured endpoint order. Network completion
            // order is deliberately absent from this phase.
            for (auto endpoint_idx = std::size_t{0};
                 endpoint_idx != endpoints.size(); ++endpoint_idx) {
              auto const& ep = endpoints[endpoint_idx];
              auto& fetched = collected[endpoint_idx];
              utl::visit(
                  ep,
                  [&](gtfs_rt_endpoint const& g) {
                    try {
                      if (fetched.error_) {
                        std::rethrow_exception(fetched.error_);
                      }
                      if (!fetched.body_) {
                        throw std::runtime_error{
                            "GTFS-RT fetch returned no "
                            "payload"};
                      }
                      add_to_group(endpoint_idx,
                                   prepare_valid_gtfsrt(endpoint_idx, g,
                                                        *fetched.body_));
                    } catch (gtfsrt_payload_exception const& e) {
                      count_payload_error(g.metrics_, e.error_);
                      if (!c.timetable_->canned_rt_) {
                        g.metrics_.updates_error_.Increment();
                      }
                      n::log(n::log_lvl::error, "motis.rt",
                             "RT PAYLOAD ERROR: tag={}, error={}", g.tag_,
                             e.what());
                      add_to_group(endpoint_idx,
                                   prepare_last_good(endpoint_idx, g));
                    } catch (std::exception const& e) {
                      g.metrics_.fetch_error_.Increment();
                      if (!c.timetable_->canned_rt_) {
                        g.metrics_.updates_error_.Increment();
                      }
                      n::log(n::log_lvl::error, "motis.rt",
                             "RT FETCH ERROR: tag={}, error={}", g.tag_,
                             e.what());
                      add_to_group(endpoint_idx,
                                   prepare_last_good(endpoint_idx, g));
                    }
                  },
                  [&](auser_endpoint const& a) {
                    if (fetched.error_ || !fetched.body_) {
                      if (fetched.error_) {
                        try {
                          std::rethrow_exception(fetched.error_);
                        } catch (std::exception const& e) {
                          n::log(n::log_lvl::error, "motis.rt",
                                 "VDV AUS FETCH ERROR: tag={}, url={}, "
                                 "error={}",
                                 a.tag_, a.ep_.url_, e.what());
                        }
                      }
                      if (!c.timetable_->canned_rt_) {
                        a.metrics_.updates_error_.Increment();
                      }
                      results[endpoint_idx] = {
                          n::rt::vdv_aus::statistics{.error_ = true}, false};
                    }
                    add_to_group(endpoint_idx,
                                 prepared_auser_update{
                                     endpoint_idx, std::move(fetched.body_)});
                  });
            }
            set_phase("feed_prepare", feed_prepare_started);

            auto const apply_prepared_gtfsrt = [&](prepared_gtfsrt_update&
                                                       prepared) {
              auto const& g =
                  std::get<gtfs_rt_endpoint>(endpoints[prepared.endpoint_idx_]);
              if (!prepared.msg_) {
                results[prepared.endpoint_idx_] = {
                    n::rt::statistics{.parser_error_ = true}, false};
                return;
              }
              try {
                auto stats = apply_gtfsrt(g, prepared, *prepared.msg_, false);
                results[prepared.endpoint_idx_] = {std::move(stats),
                                                   prepared.source_success_};
              } catch (std::exception const& e) {
                if (!c.timetable_->canned_rt_) {
                  g.metrics_.updates_error_.Increment();
                }
                n::log(n::log_lvl::error, "motis.rt",
                       "RT APPLY ERROR: tag={}, error={}", g.tag_, e.what());
                auto fallback = prepare_last_good(prepared.endpoint_idx_, g);
                if (fallback.msg_) {
                  try {
                    results[prepared.endpoint_idx_] = {
                        apply_gtfsrt(g, fallback, *fallback.msg_, true), false};
                  } catch (std::exception const& fallback_error) {
                    if (!c.timetable_->canned_rt_) {
                      g.metrics_.updates_error_.Increment();
                    }
                    n::log(n::log_lvl::error, "motis.rt",
                           "RT FALLBACK APPLY ERROR: tag={}, error={}", g.tag_,
                           fallback_error.what());
                    results[prepared.endpoint_idx_] = {
                        n::rt::statistics{.parser_error_ = true}, false};
                  } catch (...) {
                    if (!c.timetable_->canned_rt_) {
                      g.metrics_.updates_error_.Increment();
                    }
                    n::log(n::log_lvl::error, "motis.rt",
                           "RT FALLBACK APPLY ERROR: tag={}, "
                           "error=unknown",
                           g.tag_);
                    results[prepared.endpoint_idx_] = {
                        n::rt::statistics{.parser_error_ = true}, false};
                  }
                } else {
                  results[prepared.endpoint_idx_] = {
                      n::rt::statistics{.parser_error_ = true}, false};
                }
              }
            };
            auto const apply_prepared_auser =
                [&](prepared_auser_update& prepared) {
                  if (!prepared.body_) {
                    return;
                  }
                  auto const& a = std::get<auser_endpoint>(
                      endpoints[prepared.endpoint_idx_]);
                  try {
                    auto& auser = d.auser_->at(a.ep_.url_);
                    auto& target =
                        mixed_incremental_sources ? *auser_rtt : *rtt;
                    results[prepared.endpoint_idx_] = {
                        auser.consume_update(*prepared.body_, target, true)};
                  } catch (std::exception const& e) {
                    if (!c.timetable_->canned_rt_) {
                      a.metrics_.updates_error_.Increment();
                    }
                    n::log(n::log_lvl::error, "motis.rt",
                           "VDV AUS APPLY ERROR: tag={}, url={}, error={}",
                           a.tag_, a.ep_.url_, e.what());
                    results[prepared.endpoint_idx_] = {
                        n::rt::vdv_aus::statistics{.error_ = true}, false};
                  }
                };

            // Apply every prepared provider message once, serially and in
            // configured order within its dataset/source group. Mixed
            // incremental deployments first advance their persistent
            // AUSER/SIRI baseline, then overlay GTFS snapshots on a copy.
            auto const feed_apply_started = std::chrono::steady_clock::now();
            if (mixed_incremental_sources) {
              for (auto& group : groups) {
                for (auto& update : group.updates_) {
                  if (auto* prepared =
                          std::get_if<prepared_auser_update>(&update);
                      prepared != nullptr) {
                    apply_prepared_auser(*prepared);
                  }
                }
              }
              rtt = std::make_unique<n::rt_timetable>(*auser_rtt);
              for (auto& group : groups) {
                for (auto& update : group.updates_) {
                  if (auto* prepared =
                          std::get_if<prepared_gtfsrt_update>(&update);
                      prepared != nullptr) {
                    apply_prepared_gtfsrt(*prepared);
                  }
                }
              }
            } else {
              for (auto& group : groups) {
                for (auto& update : group.updates_) {
                  utl::visit(update, apply_prepared_gtfsrt,
                             apply_prepared_auser);
                }
              }
            }

            for (auto&& [ep, result] : utl::zip(endpoints, results)) {
              utl::visit(
                  ep,
                  [&](gtfs_rt_endpoint const& g) {
                    auto const& stats =
                        std::get<n::rt::statistics>(result.stats_);
                    if (!c.timetable_->canned_rt_ && result.source_success_) {
                      g.metrics_.updates_successful_.Increment();
                      g.metrics_.last_update_timestamp_.SetToCurrentTime();
                      g.metrics_.update(stats);
                    }
                    n::log(n::log_lvl::info, "motis.rt",
                           "GTFS-RT update stats for tag={}, url={}: {}",
                           g.tag_, g.ep_.url_, fmt::streamed(stats));
                  },
                  [&](auser_endpoint const& a) {
                    auto const& stats =
                        std::get<n::rt::vdv_aus::statistics>(result.stats_);
                    if (!c.timetable_->canned_rt_ && result.source_success_) {
                      a.metrics_.updates_successful_.Increment();
                      a.metrics_.last_update_timestamp_.SetToCurrentTime();
                      a.metrics_.update(stats);
                    }
                    n::log(n::log_lvl::info, "motis.rt",
                           "VDV AUS update stats for tag={}, url={}:\n{}",
                           a.tag_, a.ep_.url_, fmt::streamed(stats));
                  });
            }
            set_phase("feed_apply", feed_apply_started);

            auto pending_prediction_diagnostics =
                std::unique_ptr<vehicle_prediction_diagnostics_store>{};
            auto selected_overlays =
                std::vector<selected_vehicle_prediction_trip>{};
            auto prediction_candidates =
                std::vector<vehicle_prediction_cycle_result>{};
            auto cycle_selections = std::vector<selected_cycle_prediction>{};
            if (vehicle_history != nullptr) {
              auto const cycle_now =
                  std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count();
              vehicle_position_store->prune_before_ingested_time(
                  vehicle_matching::freshness_cutoff(
                      cycle_now, history_policy.max_age_.count()));
              auto const history_prune_started =
                  std::chrono::steady_clock::now();
              vehicle_history->prune(cycle_now, history_policy);
              history_update_cpu +=
                  std::chrono::steady_clock::now() - history_prune_started;
              history_active_vehicles.Set(
                  static_cast<double>(vehicle_history->active_histories()));
              history_observations.Set(
                  static_cast<double>(vehicle_history->observation_count()));
              history_memory_bytes.Set(static_cast<double>(
                  vehicle_history->estimated_memory_bytes()));
              history_update_seconds.Set(
                  std::chrono::duration<double>{history_update_cpu}.count());
              phase_values.at("history") =
                  std::chrono::duration<double>{history_update_cpu}.count();
              set_workload("vehicles", vehicle_position_store->all().size());
              set_workload("history_observations",
                           vehicle_history->observation_count());

              for (auto const& [_, metric] : progress_outcome_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : progress_lateral_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_outcome_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_horizon_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_error_metrics) {
                metric->Set(0.0);
              }
              auto const progress_started = std::chrono::steady_clock::now();
              auto const diagnostics = evaluate_trip_progress_diagnostics(
                  c, *d.tags_, *d.tt_, rtt.get(), d.shapes_.get(),
                  *vehicle_position_store, *vehicle_history,
                  std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count(),
                  &eta_control);
              set_workload("progress_diagnostics", diagnostics.size());
              auto lateral = std::map<std::pair<std::string, std::string>,
                                      std::vector<double>>{};
              for (auto const& diagnostic : diagnostics) {
                auto const mode =
                    diagnostic.mode_.has_value()
                        ? std::string{n::to_str(*diagnostic.mode_)}
                        : std::string{"unknown"};
                auto const outcome_key =
                    std::tuple{diagnostic.feed_, mode,
                               std::string{to_str(diagnostic.status_)}};
                auto [outcome_it, inserted] =
                    progress_outcome_metrics.try_emplace(outcome_key, nullptr);
                if (inserted) {
                  auto const& [feed, metric_mode, outcome] = outcome_key;
                  outcome_it->second =
                      &metric_families.vehicle_eta_progress_outcomes_.Add(
                          {{"feed", feed},
                           {"mode", metric_mode},
                           {"outcome", outcome}});
                }
                outcome_it->second->Increment();
                if (diagnostic.lateral_error_m_.has_value()) {
                  lateral[{diagnostic.feed_, mode}].push_back(
                      *diagnostic.lateral_error_m_);
                }
              }
              for (auto const& [feed_mode, errors] : lateral) {
                auto const sum =
                    std::accumulate(begin(errors), end(errors), 0.0);
                auto const maximum =
                    *std::max_element(begin(errors), end(errors));
                for (auto const& [statistic, value] :
                     {std::pair{"average", sum / errors.size()},
                      std::pair{"maximum", maximum}}) {
                  auto const key = std::tuple{feed_mode.first, feed_mode.second,
                                              std::string{statistic}};
                  auto [it, inserted] =
                      progress_lateral_metrics.try_emplace(key, nullptr);
                  if (inserted) {
                    auto const& [feed, mode, stat] = key;
                    it->second =
                        &metric_families
                             .vehicle_eta_progress_lateral_error_meters_.Add(
                                 {{"feed", feed},
                                  {"mode", mode},
                                  {"statistic", stat}});
                  }
                  it->second->Set(value);
                }
              }
              progress_evaluation_seconds.Set(std::chrono::duration<double>{
                  std::chrono::steady_clock::now() - progress_started}
                                                  .count());
              set_phase("progress_projection", progress_started);

              auto const gps_estimation_started =
                  std::chrono::steady_clock::now();
              prediction_candidates = evaluate_vehicle_prediction_candidates(
                  c, *d.tags_, *d.tt_, rtt.get(), d.shapes_.get(),
                  *vehicle_position_store, *vehicle_history,
                  std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count(),
                  &eta_control);
              auto& candidates = prediction_candidates;
              set_phase("gps_estimation", gps_estimation_started);
              auto const continuation_provider_started =
                  std::chrono::steady_clock::now();
              candidates = merge_retained_vehicle_prediction_continuations(
                  published_rt->vehicle_prediction_diagnostics_.get(),
                  std::move(candidates),
                  std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count(),
                  c.timetable_->vehicle_eta_->history_.max_age_seconds_);
              for (auto& provider : provider_candidates) {
                auto const existing = std::ranges::find_if(
                    candidates, [&](vehicle_prediction_cycle_result const& x) {
                      return x.batch_.transport_ ==
                                 provider.batch_.transport_ &&
                             x.trip_id_ == provider.trip_id_;
                    });
                if (existing == end(candidates)) {
                  candidates.emplace_back(std::move(provider));
                } else {
                  for (auto& prediction : provider.provider_predictions_) {
                    auto const prior = std::ranges::find_if(
                        existing->provider_predictions_,
                        [&](vehicle_stop_prediction const& candidate) {
                          return candidate.static_stop_sequence_ ==
                                     prediction.static_stop_sequence_ &&
                                 candidate.event_type_ ==
                                     prediction.event_type_;
                        });
                    if (prior == end(existing->provider_predictions_)) {
                      existing->provider_predictions_.emplace_back(
                          std::move(prediction));
                    } else {
                      *prior = std::move(prediction);
                    }
                  }
                  existing->provider_reference_timestamp_seconds_ =
                      provider.provider_reference_timestamp_seconds_;
                }
              }
              set_phase("continuation_provider_merge",
                        continuation_provider_started);
              set_workload("provider_candidates", provider_candidates.size());
              set_workload("candidates", candidates.size());
              set_workload(
                  "predicted_events",
                  std::accumulate(
                      begin(candidates), end(candidates), std::size_t{0U},
                      [](std::size_t const total,
                         vehicle_prediction_cycle_result const& candidate) {
                        return total + candidate.batch_.predictions_.size() +
                               candidate.provider_predictions_.size();
                      }));
              auto const calibration_started = std::chrono::steady_clock::now();
              if (eta_calibration.has_value()) {
                auto const calibration = eta_calibration->ingest(
                    candidates,
                    std::chrono::duration_cast<std::chrono::seconds>(
                        now.time_since_epoch())
                        .count(),
                    c.timetable_->update_interval_);
                set_workload("calibration_pending",
                             calibration.pending_entries_);
                set_workload("calibration_completed",
                             calibration.completed_entries_);
                set_workload("calibration_appended_records",
                             calibration.appended_records_);
                set_workload("calibration_appended_bytes",
                             calibration.appended_bytes_);
              }
              set_phase("calibration_capture", calibration_started);
              auto const metric_aggregation_started =
                  std::chrono::steady_clock::now();
              auto candidate_memory = std::size_t{0U};
              auto horizons = std::map<std::pair<std::string, std::string>,
                                       std::vector<std::int64_t>>{};
              using error_key =
                  std::tuple<std::string, std::string, std::string>;
              auto errors = std::map<error_key, std::vector<std::int64_t>>{};
              for (auto const& candidate : candidates) {
                auto const mode = candidate.mode_.has_value()
                                      ? std::string{n::to_str(*candidate.mode_)}
                                      : std::string{"unknown"};
                auto const outcome =
                    candidate.batch_.eligible() ? std::string{"eligible"}
                    : !candidate.provider_predictions_.empty() &&
                            !candidate.batch_.diagnostics_.rejection_
                                 .has_value()
                        ? std::string{"provider_only"}
                        : std::string{to_str(
                              *candidate.batch_.diagnostics_.rejection_)};
                auto const key =
                    std::tuple{candidate.feed_, mode, std::move(outcome)};
                auto [it, inserted] =
                    candidate_outcome_metrics.try_emplace(key, nullptr);
                if (inserted) {
                  auto const& [feed, metric_mode, metric_outcome] = key;
                  it->second =
                      &metric_families.vehicle_eta_candidate_outcomes_.Add(
                          {{"feed", feed},
                           {"mode", metric_mode},
                           {"outcome", metric_outcome}});
                }
                it->second->Increment();
                candidate_memory += candidate.batch_.estimated_memory_bytes();
                for (auto const& prediction : candidate.batch_.predictions_) {
                  horizons[{candidate.feed_, mode}].push_back(
                      prediction.horizon_seconds_);
                }
                auto& raw = errors[{candidate.feed_, mode, "seconds"}];
                raw.insert(end(raw),
                           begin(candidate.provider_raw_error_seconds_),
                           end(candidate.provider_raw_error_seconds_));
                auto& minutes = errors[{candidate.feed_, mode, "minutes"}];
                minutes.insert(end(minutes),
                               begin(candidate.provider_minute_error_),
                               end(candidate.provider_minute_error_));
              }
              for (auto const& [feed_mode, values] : horizons) {
                if (values.empty()) {
                  continue;
                }
                auto const sum = std::accumulate(begin(values), end(values),
                                                 std::int64_t{0});
                auto const maximum = *std::ranges::max_element(values);
                for (auto const& [statistic, value] :
                     {std::pair{"average",
                                static_cast<double>(sum) / values.size()},
                      std::pair{"maximum", static_cast<double>(maximum)}}) {
                  auto const key = std::tuple{feed_mode.first, feed_mode.second,
                                              std::string{statistic}};
                  auto [it, inserted] =
                      candidate_horizon_metrics.try_emplace(key, nullptr);
                  if (inserted) {
                    auto const& [feed, mode, stat] = key;
                    it->second =
                        &metric_families.vehicle_eta_candidate_horizon_seconds_
                             .Add({{"feed", feed},
                                   {"mode", mode},
                                   {"statistic", stat}});
                  }
                  it->second->Set(value);
                }
              }
              for (auto const& [base_key, values] : errors) {
                if (values.empty()) {
                  continue;
                }
                auto const signed_sum = std::accumulate(
                    begin(values), end(values), std::int64_t{0});
                auto const absolute_sum =
                    std::accumulate(begin(values), end(values), std::int64_t{0},
                                    [](auto const sum, auto const value) {
                                      return sum + std::abs(value);
                                    });
                auto const& [feed, mode, unit] = base_key;
                for (auto const& [statistic, value] :
                     {std::pair{
                          "signed_average",
                          static_cast<double>(signed_sum) / values.size()},
                      std::pair{
                          "absolute_average",
                          static_cast<double>(absolute_sum) / values.size()}}) {
                  auto const key =
                      std::tuple{feed, mode, unit, std::string{statistic}};
                  auto [it, inserted] =
                      candidate_error_metrics.try_emplace(key, nullptr);
                  if (inserted) {
                    auto const& [metric_feed, metric_mode, metric_unit, stat] =
                        key;
                    it->second =
                        &metric_families.vehicle_eta_candidate_error_.Add(
                            {{"feed", metric_feed},
                             {"mode", metric_mode},
                             {"unit", metric_unit},
                             {"statistic", stat}});
                  }
                  it->second->Set(value);
                }
              }
              candidate_memory_bytes.Set(static_cast<double>(candidate_memory));
              set_phase("metrics_aggregation", metric_aggregation_started);
              candidate_evaluation_seconds.Set(
                  phase_values.at("gps_estimation"));
              auto const selection_started = std::chrono::steady_clock::now();
              cycle_selections = select_cycle_predictions(
                  c, candidates,
                  std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count(),
                  eta_control, effective_prediction_selection_state);
              auto gps_selected_transports = std::set<n::transport>{};
              for (auto const& selected : cycle_selections) {
                if (selected.selection_.source_ ==
                    vehicle_prediction_source::kGps) {
                  gps_selected_transports.emplace(
                      selected.candidate_->batch_.transport_);
                }
              }
              for (auto const& selected : cycle_selections) {
                auto const& candidate = *selected.candidate_;
                if (selected.selection_.source_ ==
                        vehicle_prediction_source::kSchedule ||
                    !gps_selected_transports.contains(
                        candidate.batch_.transport_)) {
                  continue;
                }
                selected_overlays.push_back(
                    {.transport_ = candidate.batch_.transport_,
                     .source_ = d.tags_->get_src(candidate.feed_),
                     .trip_stop_range_ = candidate.trip_stop_range_,
                     .predictions_ = selected.selection_.predictions_});
              }
              set_phase("selection", selection_started);
              set_workload("selected_overlays", selected_overlays.size());
            } else {
              history_active_vehicles.Set(0.0);
              history_observations.Set(0.0);
              history_memory_bytes.Set(0.0);
              history_update_seconds.Set(0.0);
              progress_evaluation_seconds.Set(0.0);
              candidate_evaluation_seconds.Set(0.0);
              candidate_memory_bytes.Set(0.0);
              for (auto const& [_, metric] : progress_outcome_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : progress_lateral_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_outcome_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_horizon_metrics) {
                metric->Set(0.0);
              }
              for (auto const& [_, metric] : candidate_error_metrics) {
                metric->Set(0.0);
              }
            }

            auto const timetable_clone_started =
                std::chrono::steady_clock::now();
            auto effective_rtt = std::make_unique<n::rt_timetable>(*rtt);
            set_phase("timetable_clone", timetable_clone_started);
            auto const overlay_started = std::chrono::steady_clock::now();
            auto overlay_result = vehicle_prediction_overlay_result{};
            if (c.timetable_->vehicle_eta_.has_value()) {
              auto const& selection = c.timetable_->vehicle_eta_->selection_;
              overlay_result = apply_vehicle_prediction_overlay(
                  *d.tt_, *effective_rtt, selected_overlays,
                  {.early_departure_tolerance_seconds_ =
                       selection.early_departure_tolerance_seconds_,
                   .minute_rounding_deadband_seconds_ =
                       selection.minute_rounding_deadband_seconds_},
                  prediction_overlay_state);
            }
            set_phase("overlay", overlay_started);
            if (vehicle_history != nullptr) {
              auto const diagnostics_started = std::chrono::steady_clock::now();
              pending_prediction_diagnostics =
                  build_vehicle_prediction_diagnostics(
                      c, eta_control, prediction_candidates, cycle_selections,
                      overlay_result.applied_transports_,
                      prediction_overlay_state,
                      std::chrono::duration_cast<std::chrono::seconds>(
                          now.time_since_epoch())
                          .count());
              set_phase("diagnostics_materialization", diagnostics_started);
            }
            auto const lower_bounds_started = std::chrono::steady_clock::now();
            effective_rtt->update_lbs(*d.tt_);
            set_phase("lower_bounds", lower_bounds_started);

            // Publish the provider baseline and effective timetable together.
            auto const railviz_started = std::chrono::steady_clock::now();
            auto railviz_rt =
                std::make_unique<railviz_rt_index>(*d.tt_, *effective_rtt);
            set_phase("railviz", railviz_started);
            auto const elevators_started = std::chrono::steady_clock::now();
            auto elevators =
                c.has_elevators() && c.get_elevators()->url_
                    ? co_await update_elevators(c, d, *effective_rtt)
                    : std::move(d.rt_->e_);
            if (c.has_elevators() && c.get_elevators()->url_) {
              copy_elevator_footpaths(*effective_rtt, *rtt);
            }
            set_phase("elevators", elevators_started);
            auto const publication_started = std::chrono::steady_clock::now();
            auto new_rt = std::make_shared<rt>(
                std::move(effective_rtt), std::move(rtt), std::move(elevators),
                std::move(railviz_rt), std::move(vehicle_position_store),
                std::move(vehicle_history),
                std::move(pending_prediction_diagnostics));
            std::atomic_store(&d.rt_, std::move(new_rt));

            d.metrics_->last_update_rt_.SetToCurrentTime();
            set_phase("publication", publication_started);
          } catch (std::exception const& e) {
            n::log(n::log_lvl::error, "motis.rt",
                   "RT UPDATE CYCLE ERROR: error={}", e.what());
          } catch (...) {
            n::log(n::log_lvl::error, "motis.rt",
                   "RT UPDATE CYCLE ERROR: error=unknown");
          }

          auto const cycle_elapsed = std::chrono::steady_clock::now() - start;
          for (auto const& [phase, value] : phase_values) {
            phase_metrics.at(phase)->Set(value);
          }
          for (auto const& [workload, value] : workload_values) {
            workload_metrics.at(workload)->Set(static_cast<double>(value));
          }
          cycle_duration.Set(
              std::chrono::duration<double>{cycle_elapsed}.count());
          if (cycle_elapsed >
              std::chrono::seconds{c.timetable_->update_interval_}) {
            cycle_overrun.Increment();
          }
          cycle_completed.Increment();

          // Schedule next update.
          timer.expires_at(
              start + std::chrono::seconds{c.timetable_->update_interval_});
          co_await timer.async_wait(
              asio::redirect_error(asio::use_awaitable, ec));
          if (ec == asio::error::operation_aborted) {
            co_return;
          }
        }
      },
      boost::asio::detached);
}

}  // namespace motis
