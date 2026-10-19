#include "motis/rt/gtfsrt_service_day.h"

#include <chrono>
#include <optional>

#include "date/date.h"

#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/timetable.h"

#include "motis/timetable/time_conv.h"

namespace n = nigiri;

namespace motis {

namespace {

struct service_day_match {
  date::sys_days day_;
  std::int64_t distance_seconds_;
};

// Distance from `now` to the scheduled run of the trip on `day`: zero while
// the run operates, otherwise the gap to its first departure or last arrival.
std::optional<std::int64_t> run_distance(n::timetable const& tt,
                                         n::source_idx_t const src,
                                         transit_realtime::TripDescriptor const& td,
                                         date::sys_days const day,
                                         std::int64_t const now) {
  auto const start_time =
      td.has_start_time()
          ? std::make_optional(n::loader::gtfs::hhmm_to_min(td.start_time()))
          : std::nullopt;
  auto distance = std::optional<std::int64_t>{};
  n::rt::resolve_static_trip_id(
      day, tt, src, td.trip_id(), day, start_time,
      [&](n::rt::run const r, n::trip_idx_t) {
        auto const dep = to_seconds(
            tt.event_time(r.t_, r.stop_range_.from_, n::event_type::kDep));
        auto const arr = to_seconds(tt.event_time(
            r.t_, static_cast<n::stop_idx_t>(r.stop_range_.to_ - 1U),
            n::event_type::kArr));
        auto const d = now < dep ? dep - now : now > arr ? now - arr : 0;
        distance = distance.has_value() ? std::min(*distance, d) : d;
        return utl::continue_t::kContinue;
      });
  return distance;
}

std::optional<service_day_match> closest_service_day(
    n::timetable const& tt,
    n::source_idx_t const src,
    transit_realtime::TripDescriptor const& td,
    std::int64_t const now) {
  auto const today = std::chrono::floor<date::days>(
      std::chrono::sys_seconds{std::chrono::seconds{now}});
  auto best = std::optional<service_day_match>{};
  for (auto const day : {today - date::days{1}, today, today + date::days{1}}) {
    auto const distance = run_distance(tt, src, td, day, now);
    if (distance.has_value() &&
        (!best.has_value() || *distance < best->distance_seconds_)) {
      best = service_day_match{day, *distance};
    }
  }
  return best;
}

enum class outcome { kUnchanged, kInferred, kNotOperating };

outcome infer(n::timetable const& tt,
              n::source_idx_t const src,
              transit_realtime::TripDescriptor& td,
              std::int64_t const now,
              std::int64_t const max_distance) {
  if (!td.has_trip_id() || td.has_start_date()) {
    return outcome::kUnchanged;
  }
  auto const match = closest_service_day(tt, src, td, now);
  if (!match.has_value()) {
    return outcome::kUnchanged;
  }
  if (match->distance_seconds_ > max_distance) {
    return outcome::kNotOperating;
  }
  td.set_start_date(date::format("%Y%m%d", match->day_));
  return outcome::kInferred;
}

}  // namespace

gtfsrt_service_day_stats infer_gtfsrt_start_dates(
    n::timetable const& tt,
    n::source_idx_t const src,
    transit_realtime::FeedMessage& msg,
    std::int64_t const now_seconds,
    std::int64_t const max_distance_seconds) {
  auto stats = gtfsrt_service_day_stats{};
  auto& entities = *msg.mutable_entity();
  for (auto it = entities.begin(); it != entities.end();) {
    auto drop = false;
    if (it->has_trip_update()) {
      switch (infer(tt, src, *it->mutable_trip_update()->mutable_trip(),
                    now_seconds, max_distance_seconds)) {
        case outcome::kInferred: ++stats.inferred_; break;
        case outcome::kNotOperating:
          ++stats.dropped_trip_updates_;
          drop = true;
          break;
        case outcome::kUnchanged: break;
      }
    }
    if (!drop && it->has_vehicle() && it->vehicle().has_trip()) {
      switch (infer(tt, src, *it->mutable_vehicle()->mutable_trip(),
                    now_seconds, max_distance_seconds)) {
        case outcome::kInferred: ++stats.inferred_; break;
        case outcome::kNotOperating:
          ++stats.cleared_vehicle_trips_;
          it->mutable_vehicle()->clear_trip();
          break;
        case outcome::kUnchanged: break;
      }
    }
    it = drop ? entities.erase(it) : std::next(it);
  }
  return stats;
}

}  // namespace motis
