#include "motis/rt/vehicle_position.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_set>

#include "gtfsrt/gtfs-realtime.pb.h"

#include "utl/verify.h"

namespace gtfsrt = transit_realtime;

namespace motis::vehicle_positions {

namespace {

bool valid_pos(geo::latlng const& pos) {
  return std::isfinite(pos.lat_) && std::isfinite(pos.lng_) &&
         -90.0 <= pos.lat_ && pos.lat_ <= 90.0 && -180.0 <= pos.lng_ &&
         pos.lng_ <= 180.0 && !(pos.lat_ == 0.0 && pos.lng_ == 0.0);
}

std::optional<std::string> present_string(std::string const& s) {
  return s.empty() ? std::nullopt : std::optional{s};
}

std::optional<std::string> present_enum_name(std::string const& name) {
  return name.empty() ? std::nullopt : std::optional{name};
}

auto recency_key(vehicle_position const& position) {
  constexpr auto kMaxFutureSkew = std::int64_t{60};
  auto const ceiling =
      position.ingested_time_ >
              std::numeric_limits<std::int64_t>::max() - kMaxFutureSkew
          ? std::numeric_limits<std::int64_t>::max()
          : position.ingested_time_ + kMaxFutureSkew;
  auto const reported = position.reported_time_.has_value() &&
                                *position.reported_time_ <= ceiling
                            ? *position.reported_time_
                            : position.ingested_time_;
  return std::tuple{
      reported, position.ingested_time_, position.entity_id_};
}

void canonicalize_feed_positions(std::vector<vehicle_position>& positions) {
  std::ranges::sort(positions, std::greater{}, recency_key);
  auto entity_ids = std::unordered_set<std::string>{};
  auto vehicle_ids = std::unordered_set<std::string>{};
  entity_ids.reserve(positions.size());
  vehicle_ids.reserve(positions.size());
  std::erase_if(positions, [&](vehicle_position const& position) {
    if (!position.entity_id_.empty() &&
        !entity_ids.emplace(position.entity_id_).second) {
      return true;
    }
    return position.vehicle_.id_.has_value() &&
           !position.vehicle_.id_->empty() &&
           !vehicle_ids.emplace(*position.vehicle_.id_).second;
  });
  if (positions.size() > kMaxVehiclePositionsPerFeed) {
    positions.resize(kMaxVehiclePositionsPerFeed);
  }
}

}  // namespace

bool vehicle_viewport::contains(geo::latlng const& pos) const {
  auto const min_lat = std::min(min_.lat_, max_.lat_);
  auto const max_lat = std::max(min_.lat_, max_.lat_);
  auto const longitude_matches =
      min_.lng_ <= max_.lng_ ? min_.lng_ <= pos.lng_ && pos.lng_ <= max_.lng_
                             : min_.lng_ <= pos.lng_ || pos.lng_ <= max_.lng_;
  return min_lat <= pos.lat_ && pos.lat_ <= max_lat && longitude_matches;
}

std::vector<vehicle_position> parse_gtfsrt_vehicle_positions(
    std::string_view const feed_id,
    gtfsrt::FeedMessage const& msg,
    std::int64_t const ingested_time) {
  auto positions = std::vector<vehicle_position>{};

  for (auto const& entity : msg.entity()) {
    if ((entity.has_is_deleted() && entity.is_deleted()) ||
        !entity.has_vehicle() || !entity.vehicle().has_position()) {
      continue;
    }

    auto const& gtfs_vehicle = entity.vehicle();
    auto const& gtfs_pos = gtfs_vehicle.position();
    auto const pos = geo::latlng{gtfs_pos.latitude(), gtfs_pos.longitude()};
    if (!valid_pos(pos)) {
      continue;
    }

    auto vehicle = vehicle_position{
        .feed_id_ = std::string{feed_id},
        .entity_id_ = entity.id(),
        .reported_position_ =
            reported_position{
                .pos_ = pos,
                .bearing_ = gtfs_pos.has_bearing() &&
                                    std::isfinite(gtfs_pos.bearing()) &&
                                    0.0 <= gtfs_pos.bearing() &&
                                    gtfs_pos.bearing() < 360.0
                                ? std::optional<double>{gtfs_pos.bearing()}
                                : std::nullopt,
                .speed_mps_ = gtfs_pos.has_speed() &&
                                   std::isfinite(gtfs_pos.speed()) &&
                                   0.0 <= gtfs_pos.speed()
                                  ? std::optional<double>{gtfs_pos.speed()}
                                  : std::nullopt},
        .current_stop_sequence_ =
            gtfs_vehicle.has_current_stop_sequence()
                ? std::optional<std::uint32_t>{gtfs_vehicle
                                                   .current_stop_sequence()}
                : std::nullopt,
        .stop_id_ = present_string(gtfs_vehicle.stop_id()),
        .current_status_ =
            gtfs_vehicle.has_current_status()
                ? present_enum_name(
                      gtfsrt::VehiclePosition_VehicleStopStatus_Name(
                          gtfs_vehicle.current_status()))
                : std::nullopt,
        .occupancy_status_ =
            gtfs_vehicle.has_occupancy_status()
                ? present_enum_name(
                      gtfsrt::VehiclePosition_OccupancyStatus_Name(
                          gtfs_vehicle.occupancy_status()))
                : std::nullopt,
        .reported_time_ = gtfs_vehicle.has_timestamp() &&
                                  gtfs_vehicle.timestamp() <=
                                      static_cast<std::uint64_t>(
                                          std::numeric_limits<
                                              std::int64_t>::max())
                              ? std::optional<std::int64_t>{
                                    static_cast<std::int64_t>(
                                        gtfs_vehicle.timestamp())}
                              : std::nullopt,
        .ingested_time_ = ingested_time};

    if (gtfs_vehicle.has_vehicle()) {
      auto const& descriptor = gtfs_vehicle.vehicle();
      vehicle.vehicle_.id_ = present_string(descriptor.id());
      vehicle.vehicle_.label_ = present_string(descriptor.label());
      vehicle.vehicle_.license_plate_ =
          present_string(descriptor.license_plate());
      if (descriptor.has_wheelchair_accessible()) {
        vehicle.vehicle_.wheelchair_accessible_ =
            gtfsrt::VehicleDescriptor_WheelchairAccessible_Name(
                descriptor.wheelchair_accessible());
      }
    }

    if (gtfs_vehicle.has_trip()) {
      auto const& trip = gtfs_vehicle.trip();
      vehicle.trip_.trip_id_ = present_string(trip.trip_id());
      vehicle.trip_.start_date_ = present_string(trip.start_date());
      vehicle.trip_.start_time_ = present_string(trip.start_time());
      vehicle.trip_.route_id_ = present_string(trip.route_id());
      if (trip.has_direction_id()) {
        vehicle.trip_.direction_id_ = trip.direction_id();
      }
      if (trip.has_schedule_relationship()) {
        vehicle.trip_.schedule_relationship_ =
            gtfsrt::TripDescriptor_ScheduleRelationship_Name(
                trip.schedule_relationship());
      }
    }

    positions.emplace_back(std::move(vehicle));
  }

  canonicalize_feed_positions(positions);
  std::ranges::sort(positions, {}, [](vehicle_position const& x) {
    return std::pair{x.feed_id_, x.entity_id_};
  });
  return positions;
}

std::vector<vehicle_position> parse_gtfsrt_vehicle_positions(
    std::string_view const feed_id,
    std::string_view const protobuf_body,
    std::int64_t const ingested_time) {
  auto msg = gtfsrt::FeedMessage{};
  utl::verify(msg.ParseFromArray(protobuf_body.data(),
                                 static_cast<int>(protobuf_body.size())),
              "unable to parse GTFS-RT vehicle positions");
  return parse_gtfsrt_vehicle_positions(feed_id, msg, ingested_time);
}

void vehicle_position_store::replace_feed(
    std::string feed_id, std::vector<vehicle_position> positions) {
  for (auto& pos : positions) {
    pos.feed_id_ = feed_id;
  }
  canonicalize_feed_positions(positions);
  auto incoming_entity_ids = std::unordered_set<std::string_view>{};
  auto incoming_vehicle_ids = std::unordered_set<std::string_view>{};
  incoming_entity_ids.reserve(positions.size());
  incoming_vehicle_ids.reserve(positions.size());
  for (auto const& pos : positions) {
    incoming_entity_ids.emplace(pos.entity_id_);
    if (pos.vehicle_.id_.has_value()) {
      incoming_vehicle_ids.emplace(*pos.vehicle_.id_);
    }
  }

  auto retained = std::vector<vehicle_position>{};
  auto superseded_entity_ids = std::unordered_set<std::string>{};
  for (auto const& existing : positions_) {
    if (existing.feed_id_ != feed_id) {
      continue;
    }
    auto const superseded = existing.vehicle_.id_.has_value() &&
                            incoming_vehicle_ids.contains(*existing.vehicle_.id_);
    if (incoming_entity_ids.contains(existing.entity_id_) || superseded) {
      if (superseded) {
        superseded_entity_ids.emplace(existing.entity_id_);
      }
      continue;
    }
    auto const key = std::pair{existing.feed_id_, existing.entity_id_};
    auto const missed = missed_full_snapshot_once_.find(key);
    if (missed == end(missed_full_snapshot_once_)) {
      retained.emplace_back(existing);
      missed_full_snapshot_once_.emplace(key);
    } else {
      missed_full_snapshot_once_.erase(missed);
    }
  }
  std::erase_if(missed_full_snapshot_once_, [&](auto const& key) {
    return key.first == feed_id &&
           (incoming_entity_ids.contains(key.second) ||
            superseded_entity_ids.contains(key.second));
  });

  std::erase_if(positions_, [&](vehicle_position const& pos) {
    return pos.feed_id_ == feed_id;
  });
  positions_.insert(end(positions_), begin(positions), end(positions));
  positions_.insert(end(positions_), begin(retained), end(retained));
  bound_feed(feed_id);
  std::ranges::sort(positions_, {}, [](vehicle_position const& x) {
    return std::pair{x.feed_id_, x.entity_id_};
  });
}

void vehicle_position_store::update_feed(
    std::string feed_id,
    std::vector<vehicle_position> positions,
    std::vector<std::string> const& deleted_entity_ids) {
  for (auto& position : positions) {
    position.feed_id_ = feed_id;
  }
  canonicalize_feed_positions(positions);
  auto incoming_entity_ids = std::unordered_set<std::string_view>{};
  auto incoming_vehicle_ids = std::unordered_set<std::string_view>{};
  incoming_entity_ids.reserve(positions.size());
  incoming_vehicle_ids.reserve(positions.size());
  for (auto const& position : positions) {
    incoming_entity_ids.emplace(position.entity_id_);
    if (position.vehicle_.id_.has_value()) {
      incoming_vehicle_ids.emplace(*position.vehicle_.id_);
    }
  }
  auto deleted_ids = std::unordered_set<std::string_view>{};
  deleted_ids.reserve(deleted_entity_ids.size());
  for (auto const& id : deleted_entity_ids) {
    deleted_ids.emplace(id);
  }
  auto superseded_entity_ids = std::unordered_set<std::string>{};
  for (auto const& existing : positions_) {
    if (existing.feed_id_ == feed_id && existing.vehicle_.id_.has_value() &&
        incoming_vehicle_ids.contains(*existing.vehicle_.id_)) {
      superseded_entity_ids.emplace(existing.entity_id_);
    }
  }
  std::erase_if(missed_full_snapshot_once_, [&](auto const& key) {
    return key.first == feed_id &&
           (deleted_ids.contains(key.second) ||
            incoming_entity_ids.contains(key.second) ||
            superseded_entity_ids.contains(key.second));
  });
  std::erase_if(positions_, [&](vehicle_position const& existing) {
    if (existing.feed_id_ != feed_id) {
      return false;
    }
    return deleted_ids.contains(existing.entity_id_) ||
           incoming_entity_ids.contains(existing.entity_id_) ||
           superseded_entity_ids.contains(existing.entity_id_);
  });
  positions_.insert(end(positions_), begin(positions), end(positions));
  bound_feed(feed_id);
  std::ranges::sort(positions_, {}, [](vehicle_position const& x) {
    return std::pair{x.feed_id_, x.entity_id_};
  });
}

void vehicle_position_store::bound_feed(std::string_view const feed_id) {
  std::ranges::sort(positions_, std::greater{}, recency_key);
  auto retained = std::size_t{0U};
  std::erase_if(positions_, [&](vehicle_position const& position) {
    return position.feed_id_ == feed_id &&
           retained++ >= kMaxVehiclePositionsPerFeed;
  });
  auto entity_ids = std::unordered_set<std::string_view>{};
  for (auto const& position : positions_) {
    if (position.feed_id_ == feed_id) {
      entity_ids.emplace(position.entity_id_);
    }
  }
  std::erase_if(missed_full_snapshot_once_, [&](auto const& key) {
    return key.first == feed_id && !entity_ids.contains(key.second);
  });
}

void vehicle_position_store::prune_before_ingested_time(
    std::int64_t const cutoff) {
  std::erase_if(positions_, [&](vehicle_position const& x) {
    return x.ingested_time_ < cutoff;
  });
  std::erase_if(missed_full_snapshot_once_, [&](auto const& key) {
    return !std::ranges::binary_search(
        positions_, key, {}, [](vehicle_position const& x) {
          return std::pair{x.feed_id_, x.entity_id_};
        });
  });
}

std::vector<vehicle_position> vehicle_position_store::snapshot(
    vehicle_viewport const& viewport,
    std::optional<std::int64_t> const min_ingested_time) const {
  auto result = std::vector<vehicle_position>{};
  for (auto const& pos : positions_) {
    if ((!min_ingested_time.has_value() ||
         pos.ingested_time_ >= *min_ingested_time) &&
        viewport.contains(pos.reported_position_.pos_)) {
      result.emplace_back(pos);
    }
  }
  std::ranges::sort(result, {}, [](vehicle_position const& x) {
    return std::pair{x.feed_id_, x.entity_id_};
  });
  return result;
}

std::vector<vehicle_position> const& vehicle_position_store::all() const {
  return positions_;
}

bool vehicle_position_store::empty() const { return positions_.empty(); }

}  // namespace motis::vehicle_positions
