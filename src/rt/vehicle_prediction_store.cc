#include "motis/rt/vehicle_prediction_store.h"

#include <algorithm>
#include <ranges>
#include <tuple>
#include <utility>

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/timetable.h"

namespace motis {
namespace {

auto key(vehicle_prediction_diagnostic_entry const& x) {
  return std::tuple{x.transport_, x.trip_id_, x.static_stop_sequence_,
                    x.scheduled_timestamp_seconds_, x.event_type_};
}

auto transport_entries(
    std::vector<vehicle_prediction_diagnostic_entry> const& entries,
    nigiri::transport const transport) {
  auto const first = std::ranges::lower_bound(
      entries, transport, {}, &vehicle_prediction_diagnostic_entry::transport_);
  auto const last = std::ranges::upper_bound(
      entries, transport, {}, &vehicle_prediction_diagnostic_entry::transport_);
  return std::ranges::subrange{first, last};
}

}  // namespace

std::optional<unsigned> static_stop_sequence(
    nigiri::timetable const& tt,
    nigiri::transport_idx_t const transport,
    nigiri::trip_idx_t const trip,
    nigiri::stop_idx_t const stop) {
  if (trip == nigiri::trip_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const ranges = tt.trip_transport_ranges_[trip];
  auto const match = std::ranges::find_if(ranges, [&](auto const& entry) {
    return entry.first == transport && entry.second.from_ <= stop &&
           stop < entry.second.to_;
  });
  if (match == end(ranges)) {
    return std::nullopt;
  }
  auto const encoded = tt.trip_stop_seq_numbers_[trip];
  auto const sequences = nigiri::loader::gtfs::stop_seq_number_range{
      {encoded.data(), encoded.size()},
      static_cast<nigiri::stop_idx_t>(match->second.size())};
  auto it = begin(sequences);
  std::advance(it, static_cast<std::ptrdiff_t>(stop - match->second.from_));
  return static_cast<unsigned>(*it);
}

prediction_candidate_diagnostic resolve_effective_prediction(
    bool const is_realtime,
    std::int64_t const scheduled_timestamp_seconds,
    std::int64_t const operational_timestamp_seconds,
    vehicle_prediction_diagnostic_entry const* const diagnostic) {
  if (diagnostic != nullptr &&
      (diagnostic->effective_.source_ != vehicle_prediction_source::kSchedule ||
       !is_realtime)) {
    return diagnostic->effective_;
  }
  auto const has_realtime_prediction =
      is_realtime &&
      operational_timestamp_seconds != scheduled_timestamp_seconds;
  return {.source_ = has_realtime_prediction
                         ? vehicle_prediction_source::kProvider
                         : vehicle_prediction_source::kSchedule,
          .predicted_timestamp_seconds_ = operational_timestamp_seconds,
          .delay_seconds_ =
              operational_timestamp_seconds - scheduled_timestamp_seconds};
}

std::unique_ptr<vehicle_prediction_diagnostics_store>
vehicle_prediction_diagnostics_store::build(
    bool const enabled,
    std::vector<vehicle_prediction_diagnostic_entry> entries,
    std::int64_t const now_seconds) {
  return build(enabled, std::move(entries), now_seconds, limits{});
}

std::unique_ptr<vehicle_prediction_diagnostics_store>
vehicle_prediction_diagnostics_store::build(
    bool const enabled,
    std::vector<vehicle_prediction_diagnostic_entry> entries,
    std::int64_t const now_seconds,
    limits const policy) {
  if (!enabled) {
    return nullptr;
  }
  std::erase_if(entries, [&](auto const& x) {
    return x.transport_ == nigiri::transport::invalid() || x.trip_id_.empty() ||
           now_seconds < x.observed_at_seconds_ ||
           now_seconds - x.observed_at_seconds_ > policy.max_age_seconds_;
  });
  std::ranges::sort(entries, [](auto const& a, auto const& b) {
    if (key(a) != key(b)) {
      return key(a) < key(b);
    }
    return a.observed_at_seconds_ > b.observed_at_seconds_;
  });
  entries.erase(std::unique(begin(entries), end(entries),
                            [](auto const& a, auto const& b) {
                              return key(a) == key(b);
                            }),
                end(entries));
  if (entries.size() > policy.max_entries_) {
    std::ranges::stable_sort(entries, [](auto const& a, auto const& b) {
      return a.effective_.source_ == vehicle_prediction_source::kGps &&
             b.effective_.source_ != vehicle_prediction_source::kGps;
    });
    entries.resize(policy.max_entries_);
    std::ranges::sort(entries, {}, key);
  }
  auto store = std::make_unique<vehicle_prediction_diagnostics_store>();
  store->entries_ = std::move(entries);
  return store;
}

vehicle_prediction_diagnostic_entry const*
vehicle_prediction_diagnostics_store::find(
    nigiri::transport const transport,
    unsigned const static_stop_sequence) const {
  auto const range = transport_entries(entries_, transport);
  auto const it = std::ranges::find_if(range, [&](auto const& entry) {
    return entry.static_stop_sequence_ == static_stop_sequence;
  });
  return it == end(range) ? nullptr : &*it;
}

vehicle_prediction_diagnostic_entry const*
vehicle_prediction_diagnostics_store::find_event(
    nigiri::transport const transport,
    std::string_view const trip_id,
    unsigned const static_stop_sequence,
    std::int64_t const scheduled_timestamp_seconds,
    vehicle_prediction_event_type const event_type) const {
  auto const range = transport_entries(entries_, transport);
  auto const it = std::ranges::find_if(range, [&](auto const& entry) {
    return entry.trip_id_ == trip_id &&
           entry.static_stop_sequence_ == static_stop_sequence &&
           entry.scheduled_timestamp_seconds_ == scheduled_timestamp_seconds &&
           entry.event_type_ == event_type;
  });
  return it == end(range) ? nullptr : &*it;
}

vehicle_prediction_diagnostic_entry const*
vehicle_prediction_diagnostics_store::find_incoming_leg(
    nigiri::transport const transport, std::string_view const trip_id) const {
  auto const range = transport_entries(entries_, transport);
  auto const it = std::ranges::find_if(range, [&](auto const& entry) {
    return entry.trip_id_ == trip_id &&
           entry.selected_source_ == vehicle_prediction_source::kGps &&
           entry.context_ == vehicle_prediction_context::kIncomingBlockLeg &&
           entry.incoming_leg_provenance_.has_value();
  });
  return it == end(range) ? nullptr : &*it;
}

}  // namespace motis
