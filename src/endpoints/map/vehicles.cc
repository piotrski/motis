#include "motis/endpoints/map/vehicles.h"

#include <algorithm>
#include <chrono>

#include "net/bad_request_exception.h"
#include "net/too_many_exception.h"

#include "utl/verify.h"

#include "motis/data.h"
#include "motis/parse_location.h"
#include "motis/rt/vehicle_matching.h"
#include "motis/rt/vehicle_position.h"

namespace motis::ep {

namespace {

constexpr auto kMaxVehicleResults = std::size_t{10'000U};

}

api::VehiclePositionsResponse vehicles::operator()(
    boost::urls::url_view const& url) const {
  auto const query = api::vehicles_params{url.params()};
  auto const min = parse_location(query.min_);
  auto const max = parse_location(query.max_);
  utl::verify<net::bad_request_exception>(
      min.has_value(), "min not a coordinate: {}", query.min_);
  utl::verify<net::bad_request_exception>(
      max.has_value(), "max not a coordinate: {}", query.max_);

  auto const rt = std::atomic_load(&rt_);
  auto const rtt = rt->rtt_.get();
  auto res = api::VehiclePositionsResponse{};
  if (rt->vehicle_positions_ == nullptr) {
    return res;
  }

  auto const update_interval = config_.timetable_.has_value()
                                   ? config_.timetable_->update_interval_
                                   : 60U;
  auto const max_age = query.maxAge_.value_or(
      vehicle_matching::default_max_age(update_interval));
  utl::verify<net::bad_request_exception>(
      max_age >= 0, "maxAge must be greater than or equal to zero");
  auto const now = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  auto const cutoff = vehicle_matching::freshness_cutoff(now, max_age);
  auto const viewport =
      vehicle_positions::vehicle_viewport{.min_ = min->pos_, .max_ = max->pos_};
  res.vehicles_.reserve(
      std::min(rt->vehicle_positions_->all().size(), kMaxVehicleResults));
  auto candidate_count = std::size_t{0U};
  for (auto const& vehicle : rt->vehicle_positions_->all()) {
    if (!viewport.contains(vehicle.reported_position_.pos_) ||
        !vehicle_matching::is_fresh(vehicle, cutoff, now)) {
      continue;
    }
    utl::verify<net::too_many_exception>(candidate_count < kMaxVehicleResults,
                                         "too many vehicles");
    ++candidate_count;
    auto details = vehicle_matching::resolve_details(tags_, tt_, rtt, shapes_,
                                                     vehicle, query.language_);
    if (!query.includeUnmatched_ &&
        details.match_state_ == api::VehicleMatchStateEnum::UNMATCHED) {
      continue;
    }
    res.vehicles_.emplace_back(vehicle_matching::to_api(
        vehicle, std::move(details), query.includeShapes_));
  }
  return res;
}

}  // namespace motis::ep
