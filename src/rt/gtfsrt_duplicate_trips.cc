#include "motis/rt/gtfsrt_duplicate_trips.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "date/date.h"

#include "nigiri/loader/gtfs/stop_seq_number_encoding.h"
#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/timetable.h"

#include "motis/timetable/time_conv.h"

namespace n = nigiri;
namespace gtfsrt = transit_realtime;

namespace motis {

namespace {

// Latest VehiclePosition of one vehicle across the dataset's messages.
struct vehicle_report {
  std::string identity_;
  std::string trip_id_;
  std::string start_date_;
  std::string start_time_;
  std::int64_t timestamp_{std::numeric_limits<std::int64_t>::min()};
  std::vector<gtfsrt::VehiclePosition*> positions_;
  // Positions with this vehicle id name different vehicles (fleet numbers of
  // two fleets in one dataset): they are not joined to TripUpdates.
  bool ambiguous_{false};
};

std::string const& identity(gtfsrt::VehicleDescriptor const& v) {
  return v.license_plate().empty() ? v.id() : v.license_plate();
}

std::map<std::string, vehicle_report> vehicle_reports(
    std::span<gtfsrt::FeedMessage* const> messages) {
  auto reports = std::map<std::string, vehicle_report>{};
  for (auto* const msg : messages) {
    auto const header_timestamp =
        static_cast<std::int64_t>(msg->header().timestamp());
    for (auto& entity : *msg->mutable_entity()) {
      if (!entity.has_vehicle() || !entity.vehicle().has_vehicle() ||
          entity.vehicle().vehicle().id().empty()) {
        continue;
      }
      auto* const vp = entity.mutable_vehicle();
      auto& report = reports[vp->vehicle().id()];
      if (!report.positions_.empty() &&
          report.identity_ != identity(vp->vehicle())) {
        report.ambiguous_ = true;
      }
      report.positions_.push_back(vp);
      auto const timestamp = vp->has_timestamp()
                                 ? static_cast<std::int64_t>(vp->timestamp())
                                 : header_timestamp;
      if (report.positions_.size() == 1U || timestamp >= report.timestamp_) {
        report.identity_ = identity(vp->vehicle());
        report.trip_id_ = vp->trip().trip_id();
        report.start_date_ = vp->trip().start_date();
        report.start_time_ = vp->trip().start_time();
        report.timestamp_ = timestamp;
      }
    }
  }
  return reports;
}

// How far the first stop time update the run can place is from schedule.
std::optional<std::int64_t> schedule_deviation(n::timetable const& tt,
                                               n::source_idx_t const src,
                                               date::sys_days const today,
                                               gtfsrt::TripUpdate const& tu) {
  try {
    auto const [r, trip] =
        n::rt::gtfsrt_resolve_run(today, tt, nullptr, src, tu.trip());
    if (!r.valid() || !r.is_scheduled() || trip == n::trip_idx_t::invalid()) {
      return std::nullopt;
    }
    auto const n_stops = static_cast<n::stop_idx_t>(r.stop_range_.size());
    auto sequences = std::vector<unsigned>{};
    for (auto const s : n::loader::gtfs::stop_seq_number_range{
             {tt.trip_stop_seq_numbers_[trip]}, n_stops}) {
      sequences.push_back(s);
    }
    auto const location_seq =
        tt.route_location_seq_[tt.transport_route_[r.t_.t_idx_]];
    auto next = n::stop_idx_t{0U};
    for (auto const& stu : tu.stop_time_update()) {
      auto i = next;
      for (; i != n_stops; ++i) {
        auto const location =
            n::stop{location_seq[r.stop_range_.from_ + i]}.location_idx();
        if (stu.has_stop_sequence()
                ? stu.stop_sequence() == sequences[i]
                : stu.has_stop_id() &&
                      stu.stop_id() == tt.locations_.ids_[location].view()) {
          break;
        }
      }
      if (i == n_stops) {
        continue;
      }
      next = i;
      auto const stop_idx = static_cast<n::stop_idx_t>(r.stop_range_.from_ + i);
      auto const deviation =
          [&](gtfsrt::TripUpdate_StopTimeEvent const& ev,
              n::event_type const type) -> std::optional<std::int64_t> {
        if (ev.has_delay()) {
          return std::abs(static_cast<std::int64_t>(ev.delay()));
        }
        if (ev.has_time()) {
          return std::abs(ev.time() -
                          to_seconds(tt.event_time(r.t_, stop_idx, type)));
        }
        return std::nullopt;
      };
      if (stu.has_arrival() && i != 0U) {
        if (auto const d = deviation(stu.arrival(), n::event_type::kArr)) {
          return d;
        }
      }
      if (stu.has_departure() && i + 1U != n_stops) {
        if (auto const d = deviation(stu.departure(), n::event_type::kDep)) {
          return d;
        }
      }
    }
  } catch (...) {
  }
  return std::nullopt;
}

// A TripUpdate entity: its message and index.
struct entity_ref {
  gtfsrt::FeedMessage* msg_;
  int idx_;
};

struct run_update {
  entity_ref entity_;
  std::string identity_;
  vehicle_report* vehicle_{nullptr};
  bool fresh_{false};
  bool names_run_{false};
  bool on_run_{false};
  std::int64_t deviation_{};
};

// A run is (trip id, start date, start time), across all messages of the
// dataset (several endpoints can report the same run).
using run_key = std::tuple<std::string, std::string, std::string>;

// The run of a SCHEDULED TripUpdate.
std::optional<run_key> scheduled_run(gtfsrt::FeedEntity const& entity) {
  if (!entity.has_trip_update() ||
      (entity.has_is_deleted() && entity.is_deleted())) {
    return std::nullopt;
  }
  auto const& td = entity.trip_update().trip();
  if (td.trip_id().empty() ||
      td.schedule_relationship() !=
          gtfsrt::TripDescriptor_ScheduleRelationship_SCHEDULED) {
    return std::nullopt;
  }
  return run_key{td.trip_id(), td.start_date(), td.start_time()};
}

}  // namespace

bool has_duplicate_trip_updates(
    std::span<gtfsrt::FeedMessage const* const> messages) {
  auto runs = std::set<run_key>{};
  for (auto const* const msg : messages) {
    for (auto const& entity : msg->entity()) {
      if (auto key = scheduled_run(entity);
          key.has_value() && !runs.emplace(std::move(*key)).second) {
        return true;
      }
    }
  }
  return false;
}

duplicate_trip_stats split_duplicate_trip_updates(
    n::timetable const& tt,
    n::source_idx_t const src,
    std::span<gtfsrt::FeedMessage* const> messages,
    std::int64_t const now_seconds,
    std::int64_t const max_gps_age_seconds) {
  auto stats = duplicate_trip_stats{};
  auto reports = vehicle_reports(messages);
  auto const today = std::chrono::floor<date::days>(
      std::chrono::sys_seconds{std::chrono::seconds{now_seconds}});
  auto runs = std::map<run_key, std::vector<entity_ref>>{};
  for (auto* const msg : messages) {
    for (auto i = 0; i != msg->entity_size(); ++i) {
      if (auto key = scheduled_run(msg->entity(i)); key.has_value()) {
        runs[std::move(*key)].push_back({msg, i});
      }
    }
  }

  // Removed only at the end: `reports` and `runs` point into the messages.
  auto dropped = std::map<gtfsrt::FeedMessage*, std::vector<int>>{};
  for (auto const& [key, entities] : runs) {
    if (entities.size() < 2U) {
      continue;
    }
    auto const& [trip_id, start_date, start_time] = key;
    auto const names_run = [&](std::string const& vp_trip_id,
                               std::string const& vp_start_date,
                               std::string const& vp_start_time) {
      return vp_trip_id == trip_id &&
             (vp_start_date.empty() || start_date.empty() ||
              vp_start_date == start_date) &&
             (vp_start_time.empty() || start_time.empty() ||
              vp_start_time == start_time);
    };
    auto const names_this_run = [&](gtfsrt::VehiclePosition const& vp) {
      return names_run(vp.trip().trip_id(), vp.trip().start_date(),
                       vp.trip().start_time());
    };

    auto updates = std::vector<run_update>{};
    for (auto const e : entities) {
      auto const& tu = e.msg_->entity(e.idx_).trip_update();
      auto& u = updates.emplace_back(run_update{.entity_ = e});
      if (!tu.vehicle().id().empty()) {
        if (auto const it = reports.find(tu.vehicle().id());
            it != end(reports) && !it->second.ambiguous_) {
          u.vehicle_ = &it->second;
        }
      }
      u.identity_ = !tu.vehicle().license_plate().empty()
                        ? tu.vehicle().license_plate()
                    : u.vehicle_ != nullptr ? u.vehicle_->identity_
                                            : tu.vehicle().id();
      u.fresh_ = u.vehicle_ != nullptr &&
                 u.vehicle_->timestamp_ >= now_seconds - max_gps_age_seconds;
      u.names_run_ = u.vehicle_ != nullptr &&
                     names_run(u.vehicle_->trip_id_, u.vehicle_->start_date_,
                               u.vehicle_->start_time_);
      u.on_run_ = u.fresh_ && u.names_run_;
      u.deviation_ = schedule_deviation(tt, src, today, tu)
                         .value_or(std::numeric_limits<std::int64_t>::max());
    }
    if (std::ranges::all_of(
            updates, [](run_update const& u) { return u.identity_.empty(); })) {
      continue;  // the feed does not say which vehicles these are
    }
    ++stats.trips_;

    auto const any_on_run = std::ranges::any_of(updates, &run_update::on_run_);
    auto const any_fresh = std::ranges::any_of(updates, &run_update::fresh_);
    auto const eligible = [&](run_update const& u) {
      return any_on_run ? u.on_run_ : !any_fresh || u.fresh_;
    };
    auto keeper = static_cast<run_update const*>(nullptr);
    for (auto const& u : updates) {
      if (eligible(u) &&
          (keeper == nullptr || u.deviation_ < keeper->deviation_)) {
        keeper = &u;
      }
    }

    auto used = std::vector<std::string>{keeper->identity_};
    for (auto const& u : updates) {
      if (&u == keeper) {
        continue;
      }
      auto const same_vehicle =
          u.identity_.empty() ||
          std::ranges::find(used, u.identity_) != end(used);
      if (!any_on_run || !u.on_run_ || same_vehicle) {
        dropped[u.entity_.msg_].push_back(u.entity_.idx_);
        // A vehicle whose TripUpdate is dropped but whose old position still
        // names the run must not predict the run from GPS either. The route
        // stays, so the map still shows its line.
        if (!same_vehicle && u.names_run_ && !u.fresh_) {
          for (auto* const vp : u.vehicle_->positions_) {
            if (names_this_run(*vp)) {
              auto& td = *vp->mutable_trip();
              td.clear_trip_id();
              td.clear_start_date();
              td.clear_start_time();
            }
          }
          u.vehicle_->trip_id_.clear();
        }
        continue;
      }
      used.push_back(u.identity_);
      auto const copy_id = trip_id + "~" + u.identity_;
      auto& entity = *u.entity_.msg_->mutable_entity(u.entity_.idx_);
      entity.set_id(entity.id() + "~" + u.identity_);
      auto& tu = *entity.mutable_trip_update();
      tu.mutable_trip()->set_schedule_relationship(
          gtfsrt::TripDescriptor_ScheduleRelationship_DUPLICATED);
      tu.mutable_trip_properties()->set_trip_id(copy_id);
      for (auto* const vp : u.vehicle_->positions_) {
        if (names_this_run(*vp)) {
          vp->mutable_trip()->set_trip_id(copy_id);
        }
      }
      u.vehicle_->trip_id_ = copy_id;
      ++stats.copies_;
    }
  }

  for (auto& [msg, indices] : dropped) {
    std::ranges::sort(indices, std::greater{});
    for (auto const i : indices) {
      msg->mutable_entity()->DeleteSubrange(i, 1);
    }
    stats.dropped_ += indices.size();
  }
  return stats;
}

}  // namespace motis
