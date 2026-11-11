#include "motis/rt/gtfsrt_duplicate_trips.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
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
  std::int64_t timestamp_{std::numeric_limits<std::int64_t>::min()};
  std::vector<gtfsrt::VehiclePosition*> positions_;
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
      report.positions_.push_back(vp);
      auto const timestamp = vp->has_timestamp()
                                 ? static_cast<std::int64_t>(vp->timestamp())
                                 : header_timestamp;
      if (timestamp >= report.timestamp_) {
        report.identity_ = identity(vp->vehicle());
        report.trip_id_ = vp->trip().trip_id();
        report.start_date_ = vp->trip().start_date();
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

struct run_update {
  int entity_idx_;
  std::string identity_;
  vehicle_report* vehicle_{nullptr};
  bool fresh_{false};
  bool on_run_{false};
  std::int64_t deviation_{};
};

}  // namespace

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
  // Removed only at the end: `reports` points into the messages.
  auto dropped = std::map<gtfsrt::FeedMessage*, std::vector<int>>{};

  for (auto* const msg : messages) {
    using run_key = std::tuple<std::string, std::string, std::string>;
    auto runs = std::map<run_key, std::vector<int>>{};
    for (auto i = 0; i != msg->entity_size(); ++i) {
      auto const& entity = msg->entity(i);
      if (!entity.has_trip_update() ||
          (entity.has_is_deleted() && entity.is_deleted())) {
        continue;
      }
      auto const& td = entity.trip_update().trip();
      if (td.trip_id().empty() ||
          td.schedule_relationship() !=
              gtfsrt::TripDescriptor_ScheduleRelationship_SCHEDULED) {
        continue;
      }
      runs[{td.trip_id(), td.start_date(), td.start_time()}].push_back(i);
    }

    // Copies from an earlier pass (a last-good snapshot) keep their vehicle's
    // position on the copy when only the positions feed is new.
    for (auto const& entity : msg->entity()) {
      if (!entity.has_trip_update() ||
          entity.trip_update().trip().schedule_relationship() !=
              gtfsrt::TripDescriptor_ScheduleRelationship_DUPLICATED) {
        continue;
      }
      auto const& tu = entity.trip_update();
      auto const it = reports.find(tu.vehicle().id());
      if (it == end(reports) ||
          tu.trip_properties().trip_id() !=
              tu.trip().trip_id() + "~" + it->second.identity_) {
        continue;
      }
      for (auto* const vp : it->second.positions_) {
        if (vp->trip().trip_id() == tu.trip().trip_id()) {
          vp->mutable_trip()->set_trip_id(tu.trip_properties().trip_id());
        }
      }
    }

    for (auto const& [key, entity_indices] : runs) {
      if (entity_indices.size() < 2U) {
        continue;
      }
      auto const& trip_id = std::get<0>(key);
      auto updates = std::vector<run_update>{};
      for (auto const i : entity_indices) {
        auto const& tu = msg->entity(i).trip_update();
        auto& u = updates.emplace_back(run_update{.entity_idx_ = i});
        if (!tu.vehicle().id().empty()) {
          if (auto const it = reports.find(tu.vehicle().id());
              it != end(reports)) {
            u.vehicle_ = &it->second;
          }
        }
        u.identity_ = !tu.vehicle().license_plate().empty()
                          ? tu.vehicle().license_plate()
                      : u.vehicle_ != nullptr ? u.vehicle_->identity_
                                              : tu.vehicle().id();
        u.fresh_ = u.vehicle_ != nullptr &&
                   u.vehicle_->timestamp_ >= now_seconds - max_gps_age_seconds;
        u.on_run_ =
            u.fresh_ &&
            (u.vehicle_->trip_id_ == trip_id ||
             u.vehicle_->trip_id_ == trip_id + "~" + u.identity_) &&
            (u.vehicle_->start_date_.empty() || std::get<1>(key).empty() ||
             u.vehicle_->start_date_ == std::get<1>(key));
        u.deviation_ = schedule_deviation(tt, src, today, tu)
                           .value_or(std::numeric_limits<std::int64_t>::max());
      }
      if (std::ranges::all_of(updates, [](run_update const& u) {
            return u.identity_.empty();
          })) {
        continue;  // the feed does not say which vehicles these are
      }
      ++stats.trips_;

      auto const any_on_run =
          std::ranges::any_of(updates, &run_update::on_run_);
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
        if (!any_on_run || !u.on_run_ || u.identity_.empty() ||
            std::ranges::find(used, u.identity_) != end(used)) {
          dropped[msg].push_back(u.entity_idx_);
          continue;
        }
        used.push_back(u.identity_);
        auto const copy_id = trip_id + "~" + u.identity_;
        auto& entity = *msg->mutable_entity(u.entity_idx_);
        entity.set_id(entity.id() + "~" + u.identity_);
        auto& tu = *entity.mutable_trip_update();
        tu.mutable_trip()->set_schedule_relationship(
            gtfsrt::TripDescriptor_ScheduleRelationship_DUPLICATED);
        tu.mutable_trip_properties()->set_trip_id(copy_id);
        for (auto* const vp : u.vehicle_->positions_) {
          if (vp->trip().trip_id() == trip_id) {
            vp->mutable_trip()->set_trip_id(copy_id);
          }
        }
        u.vehicle_->trip_id_ = copy_id;
        ++stats.copies_;
      }
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
