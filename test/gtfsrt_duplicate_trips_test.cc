#include "gtest/gtest.h"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "date/date.h"

#ifdef NO_DATA
#undef NO_DATA
#endif
#include "gtfsrt/gtfs-realtime.pb.h"

#include "nigiri/rt/create_rt_timetable.h"
#include "nigiri/rt/frun.h"
#include "nigiri/rt/gtfsrt_resolve_run.h"
#include "nigiri/rt/gtfsrt_update.h"
#include "nigiri/rt/rt_timetable.h"
#include "nigiri/rt/util.h"
#include "nigiri/timetable.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/import.h"
#include "motis/rt/gtfsrt_duplicate_trips.h"
#include "motis/rt/passed_stops.h"

namespace n = nigiri;
namespace gtfsrt = transit_realtime;
using namespace motis;
using namespace std::chrono_literals;

// ZTP Kraków line 139, 2026-11-10: brygada 139-05 ran with two buses, BH411
// (TripUpdates name it by fleet number "411") and PR773. Both sent
// TripUpdates for the same trips. The fixture holds the archived feeds of that
// afternoon, trimmed to the block's three trips (…074, …075, …076). Feed
// file names are UTC times.

namespace {

constexpr auto kDir = "test/resources/ztp-139-2026-11-10";
constexpr auto kT075 = "20261109_1_130082075_11";
constexpr auto kT076 = "20261109_1_130082076_12";
constexpr auto kCopy075 = "20261109_1_130082075_11~PR773";

gtfsrt::FeedMessage load(std::string const& name) {
  auto in = std::ifstream{std::string{kDir} + "/rt/" + name + ".json"};
  auto json = std::stringstream{};
  json << in.rdbuf();
  auto msg = gtfsrt::FeedMessage{};
  EXPECT_TRUE(msg.ParseFromString(n::rt::json_to_protobuf(json.str()))) << name;
  return msg;
}

struct feeds {
  explicit feeds(std::string const& time)
      : tu_{load("tu-" + time)}, vp_{load("vp-" + time)} {}

  std::int64_t now() const {
    return static_cast<std::int64_t>(tu_.header().timestamp());
  }

  // SCHEDULED TripUpdates of a trip (copies keep the trip id they copy).
  std::vector<gtfsrt::TripUpdate const*> trip_updates(
      std::string const& trip_id) const {
    auto out = std::vector<gtfsrt::TripUpdate const*>{};
    for (auto const& e : tu_.entity()) {
      if (e.has_trip_update() && e.trip_update().trip().trip_id() == trip_id &&
          e.trip_update().trip().schedule_relationship() ==
              gtfsrt::TripDescriptor_ScheduleRelationship_SCHEDULED) {
        out.push_back(&e.trip_update());
      }
    }
    return out;
  }

  std::string vehicle_trip(std::string const& vehicle_id) const {
    for (auto const& e : vp_.entity()) {
      if (e.has_vehicle() && e.vehicle().vehicle().id() == vehicle_id) {
        return e.vehicle().trip().trip_id();
      }
    }
    return "";
  }

  gtfsrt::FeedMessage tu_, vp_;
};

struct fixture {
  explicit fixture(std::string_view const path) {
    auto ec = std::error_code{};
    std::filesystem::remove_all(path, ec);
    auto const c =
        config{.timetable_ =
                   config::timetable{
                       .first_day_ = "2026-11-09",
                       .num_days_ = 3,
                       .datasets_ = {{"test",
                                      {.path_ = std::string{kDir} + "/gtfs"}}}},
               .street_routing_ = false};
    import(c, path);
    d_.emplace(path, c);
  }

  n::timetable const& tt() const { return *d_->tt_; }

  duplicate_trip_stats split(feeds& f,
                             std::optional<std::int64_t> now = std::nullopt) {
    auto messages = std::array{&f.tu_, &f.vp_};
    return split_duplicate_trip_updates(tt(), n::source_idx_t{0}, messages,
                                        now.value_or(f.now()));
  }

  struct cycle {
    std::unique_ptr<n::rt_timetable> rtt_;
    passed_stop_store store_;
  };

  // One realtime cycle as rt_update runs it: split, then nigiri, then the
  // passed-stop update against the previous cycle.
  cycle run(feeds& f, cycle const* previous) {
    split(f);
    auto const now = date::sys_seconds{std::chrono::seconds{f.now()}};
    auto const today = std::chrono::floor<date::days>(now);
    auto rtt = std::make_unique<n::rt_timetable>(
        n::rt::create_rt_timetable(tt(), today));
    n::rt::gtfsrt_update_msg(tt(), *rtt, n::source_idx_t{0}, "test", f.tu_);
    auto store = update_passed_stops(
        {.tt_ = tt(),
         .tags_ = *d_->tags_,
         .today_ = today,
         .now_seconds_ = f.now(),
         .previous_ = previous == nullptr ? nullptr : &previous->store_,
         .previous_rtt_ = previous == nullptr ? nullptr : previous->rtt_.get()},
        *rtt, trip_update_positions(n::source_idx_t{0}, f.tu_));
    return {std::move(rtt), std::move(store)};
  }

  n::rt::run resolve(cycle const& c, std::string const& trip_id) const {
    auto td = gtfsrt::TripDescriptor{};
    td.set_trip_id(trip_id);
    return n::rt::gtfsrt_resolve_run(
               date::sys_days{date::year{2026} / date::November / 10}, tt(),
               c.rtt_.get(), n::source_idx_t{0}, td)
        .first;
  }

  // Unix time of a stop's arrival in a cycle's realtime timetable.
  std::int64_t arrival(cycle const& c,
                       std::string const& trip_id,
                       n::stop_idx_t const stop) const {
    auto const fr = n::rt::frun{tt(), c.rtt_.get(), resolve(c, trip_id)};
    return std::chrono::duration_cast<std::chrono::seconds>(
               fr[stop].time(n::event_type::kArr).time_since_epoch())
        .count();
  }

  std::optional<data> d_;
};

std::string const& vehicle(gtfsrt::TripUpdate const* tu) {
  return tu->vehicle().id();
}

// Minute precision: nigiri stores minutes.
constexpr std::int64_t minute(std::int64_t const unix) {
  return unix / 60 * 60;
}

// Pushes BH411's TripUpdate for …075 an hour late, so PR773 is closer to
// schedule.
void delay_bh411(feeds& f) {
  for (auto& e : *f.tu_.mutable_entity()) {
    if (e.trip_update().trip().trip_id() == kT075 &&
        e.trip_update().vehicle().id() == "411") {
      for (auto& stu : *e.mutable_trip_update()->mutable_stop_time_update()) {
        stu.mutable_arrival()->set_time(stu.arrival().time() + 3600);
        stu.mutable_departure()->set_time(stu.departure().time() + 3600);
      }
    }
  }
}

}  // namespace

TEST(gtfsrt_duplicate_trips, second_vehicle_gets_its_own_run) {
  auto t = fixture{"test/duplicate-trips-split"};
  auto f = feeds{"1620"};
  ASSERT_EQ(2U, f.trip_updates(kT075).size());

  auto const c = t.run(f, nullptr);

  // BH411 is on …075 and 9 min late; PR773 is behind it on the same trip,
  // 31 min late. BH411 keeps the trip and PR773 gets a copy.
  auto const scheduled = f.trip_updates(kT075);
  ASSERT_EQ(1U, scheduled.size());
  EXPECT_EQ("411", vehicle(scheduled[0]));
  EXPECT_EQ(kT075, f.vehicle_trip("411"));
  EXPECT_EQ(kCopy075, f.vehicle_trip("PR773"));

  // Dobrego Pasterza (stop 24) has BH411's time, Teatr Ludowy (stop 7) on
  // the copy has PR773's.
  EXPECT_EQ(minute(1794327666), t.arrival(c, kT075, 23U));
  auto const copy = t.resolve(c, kCopy075);
  ASSERT_TRUE(copy.valid());
  ASSERT_TRUE(copy.is_rt());
  EXPECT_NE(t.resolve(c, kT075).rt_, copy.rt_);
  EXPECT_EQ(minute(1794327592), t.arrival(c, kCopy075, 6U));
  // The copy keeps the static schedule, so it shows its own delay.
  auto const copy_fr = n::rt::frun{t.tt(), c.rtt_.get(), copy};
  // 16:19:52 for 15:49, in whole minutes.
  EXPECT_EQ(30min, copy_fr[6].delay(n::event_type::kArr));
  EXPECT_EQ(35U, copy_fr.size());  // …075's stops, not the whole block

  // Both buses forecast …076, neither is on it yet: one forecast stays.
  EXPECT_EQ(1U, f.trip_updates(kT076).size());
}

TEST(gtfsrt_duplicate_trips, forecast_of_a_vehicle_on_another_run_is_dropped) {
  auto t = fixture{"test/duplicate-trips-forecast"};
  // 16:00: BH411 is on …075, PR773 still on …074 with a forecast for …075.
  auto f = feeds{"1600"};
  ASSERT_EQ(2U, f.trip_updates(kT075).size());
  auto const stats = t.split(f);
  EXPECT_EQ(0U, stats.copies_);
  auto const left = f.trip_updates(kT075);
  ASSERT_EQ(1U, left.size());
  EXPECT_EQ("411", vehicle(left[0]));
}

TEST(gtfsrt_duplicate_trips, ghost_vehicle_is_dropped) {
  auto t = fixture{"test/duplicate-trips-ghost"};
  auto f = feeds{"1620"};
  // PR773 stops reporting its position: its TripUpdate no longer counts.
  auto& entities = *f.vp_.mutable_entity();
  for (auto i = 0; i != entities.size(); ++i) {
    if (entities[i].vehicle().vehicle().id() == "PR773") {
      entities.DeleteSubrange(i, 1);
      break;
    }
  }
  auto const stats = t.split(f);
  EXPECT_EQ(0U, stats.copies_);
  auto const left = f.trip_updates(kT075);
  ASSERT_EQ(1U, left.size());
  EXPECT_EQ("411", vehicle(left[0]));
}

TEST(gtfsrt_duplicate_trips, stale_gps_keeps_the_closest_to_schedule) {
  auto t = fixture{"test/duplicate-trips-stale"};
  auto f = feeds{"1620"};
  // All positions are 10 min old: nobody is on the run, the TripUpdate
  // closest to schedule stays and no copy is made.
  auto const stats = t.split(f, f.now() + 600);
  EXPECT_EQ(0U, stats.copies_);
  auto const left = f.trip_updates(kT075);
  ASSERT_EQ(1U, left.size());
  EXPECT_EQ("411", vehicle(left[0]));
  // PR773's old position no longer names the run it was dropped from.
  EXPECT_EQ(kT075, f.vehicle_trip("411"));
  EXPECT_EQ("", f.vehicle_trip("PR773"));
}

TEST(gtfsrt_duplicate_trips, copy_is_named_by_licence_plate) {
  auto t = fixture{"test/duplicate-trips-plate"};
  auto f = feeds{"1620"};
  // PR773 keeps the trip. The TripUpdate names BH411 "411"; its copy takes
  // the plate from its position.
  delay_bh411(f);
  t.split(f);
  EXPECT_EQ(kT075, f.vehicle_trip("PR773"));
  EXPECT_EQ(std::string{kT075} + "~BH411", f.vehicle_trip("411"));
}

TEST(gtfsrt_duplicate_trips, split_is_idempotent) {
  auto t = fixture{"test/duplicate-trips-idempotent"};
  auto f = feeds{"1620"};
  auto const first = t.split(f);
  auto const tu = f.tu_.SerializeAsString();
  auto const vp = f.vp_.SerializeAsString();
  auto const again = t.split(f);
  EXPECT_EQ(first.copies_, again.copies_);
  EXPECT_EQ(0U, again.dropped_);
  EXPECT_EQ(tu, f.tu_.SerializeAsString());
  EXPECT_EQ(vp, f.vp_.SerializeAsString());

  // A last-good TripUpdates snapshot with a fresh positions feed: the copy's
  // vehicle is moved onto the copy again.
  auto fresh = feeds{"1620"};
  fresh.tu_ = f.tu_;
  t.split(fresh);
  EXPECT_EQ(kCopy075, fresh.vehicle_trip("PR773"));
}

// A last-good positions snapshot was saved after the split. With fresh
// TripUpdates the decision is made again, not inherited from the snapshot.
TEST(gtfsrt_duplicate_trips, cached_positions_follow_a_new_keeper) {
  auto t = fixture{"test/duplicate-trips-cached-positions"};
  auto cached = feeds{"1620"};
  t.split(cached);
  ASSERT_EQ(kCopy075, cached.vehicle_trip("PR773"));

  // PR773 now keeps the trip, BH411 gets the copy.
  auto f = feeds{"1620"};
  delay_bh411(f);
  f.vp_ = cached.vp_;
  t.split(f);
  EXPECT_EQ(kT075, f.vehicle_trip("PR773"));
  EXPECT_EQ(std::string{kT075} + "~BH411", f.vehicle_trip("411"));

  // Only PR773 still sends a TripUpdate: its position names the trip again.
  auto alone = feeds{"1620"};
  auto& entities = *alone.tu_.mutable_entity();
  for (auto i = 0; i != entities.size(); ++i) {
    if (entities[i].trip_update().trip().trip_id() == kT075 &&
        entities[i].trip_update().vehicle().id() == "411") {
      entities.DeleteSubrange(i, 1);
      break;
    }
  }
  alone.vp_ = cached.vp_;
  t.split(alone);
  EXPECT_EQ(kT075, alone.vehicle_trip("PR773"));
}

// PR773's position is 200 s old: too old to keep its TripUpdate, but young
// enough for GPS predictions. It must not predict the run either.
TEST(gtfsrt_duplicate_trips, dropped_vehicle_leaves_the_run) {
  auto t = fixture{"test/duplicate-trips-dropped-position"};
  auto f = feeds{"1620"};
  for (auto& e : *f.vp_.mutable_entity()) {
    if (e.vehicle().vehicle().id() == "PR773") {
      e.mutable_vehicle()->set_timestamp(
          static_cast<std::uint64_t>(f.now() - 200));
    }
  }
  t.split(f);
  auto const left = f.trip_updates(kT075);
  ASSERT_EQ(1U, left.size());
  EXPECT_EQ("411", vehicle(left[0]));
  EXPECT_EQ("", f.vehicle_trip("PR773"));
}

// Two endpoints of one dataset each send one of the TripUpdates.
TEST(gtfsrt_duplicate_trips, trip_updates_of_two_messages_are_split) {
  auto t = fixture{"test/duplicate-trips-two-messages"};
  auto f = feeds{"1620"};
  auto second = gtfsrt::FeedMessage{};
  *second.mutable_header() = f.tu_.header();
  auto& entities = *f.tu_.mutable_entity();
  for (auto i = 0; i != entities.size(); ++i) {
    if (entities[i].trip_update().trip().trip_id() == kT075 &&
        entities[i].trip_update().vehicle().id() == "PR773") {
      *second.add_entity() = entities[i];
      entities.DeleteSubrange(i, 1);
      break;
    }
  }
  auto messages = std::array{&f.tu_, &second, &f.vp_};
  auto const stats = split_duplicate_trip_updates(t.tt(), n::source_idx_t{0},
                                                  messages, f.now());
  EXPECT_EQ(1U, stats.copies_);
  ASSERT_EQ(1, second.entity_size());
  EXPECT_EQ(kCopy075,
            second.entity(0).trip_update().trip_properties().trip_id());
  EXPECT_EQ(kCopy075, f.vehicle_trip("PR773"));
}

// Fleet number 411 is also another fleet's vehicle in the same dataset: the
// TripUpdate naming "411" cannot be joined to a position.
TEST(gtfsrt_duplicate_trips, ambiguous_fleet_number_is_not_joined) {
  auto t = fixture{"test/duplicate-trips-ambiguous"};
  auto f = feeds{"1620"};
  auto& other = *f.vp_.add_entity();
  other.set_id("tram_411");
  other.mutable_vehicle()->mutable_vehicle()->set_id("411");
  other.mutable_vehicle()->mutable_vehicle()->set_license_plate("RP411");
  other.mutable_vehicle()->mutable_trip()->set_trip_id("other");
  other.mutable_vehicle()->set_timestamp(static_cast<std::uint64_t>(f.now()));
  auto const stats = t.split(f);
  EXPECT_EQ(0U, stats.copies_);
  auto const left = f.trip_updates(kT075);
  ASSERT_EQ(1U, left.size());
  EXPECT_EQ("PR773", vehicle(left[0]));
}

TEST(gtfsrt_duplicate_trips, trip_updates_without_vehicles_are_kept) {
  auto t = fixture{"test/duplicate-trips-no-vehicle"};
  auto f = feeds{"1620"};
  for (auto& e : *f.tu_.mutable_entity()) {
    e.mutable_trip_update()->clear_vehicle();
  }
  auto const before = f.tu_.SerializeAsString();
  auto const stats = t.split(f);
  EXPECT_EQ(0U, stats.trips_);
  EXPECT_EQ(before, f.tu_.SerializeAsString());
}

TEST(gtfsrt_duplicate_trips, start_time_tells_runs_apart) {
  auto t = fixture{"test/duplicate-trips-start-time"};
  auto f = feeds{"1620"};
  auto n_075 = 0;
  for (auto& e : *f.tu_.mutable_entity()) {
    if (e.trip_update().trip().trip_id() == kT075) {
      e.mutable_trip_update()->mutable_trip()->set_start_time(
          n_075++ == 0 ? "16:40:00" : "17:40:00");
    }
  }
  t.split(f);
  EXPECT_EQ(2U, f.trip_updates(kT075).size());
}

// The whole afternoon: once split, the passed stops of …075 follow one bus.
TEST(gtfsrt_duplicate_trips, replay_passed_stops_follow_one_vehicle) {
  auto t = fixture{"test/duplicate-trips-replay"};
  auto previous = std::optional<fixture::cycle>{};
  auto last_first_covered = n::stop_idx_t{0U};
  for (auto const time :
       {"1520", "1530", "1540", "1550", "1600", "1610", "1620", "1630", "1640",
        "1645", "1650", "1700", "1710"}) {
    SCOPED_TRACE(time);
    auto f = feeds{time};
    auto c = t.run(f, previous.has_value() ? &*previous : nullptr);

    auto const updates = f.trip_updates(kT075);
    EXPECT_LE(updates.size(), 1U);
    auto const r = t.resolve(c, kT075);
    for (auto const& [key, first] : c.store_.first_covered_) {
      // From 16:50 PR773, the bus behind, has the trip to itself.
      if (key.transport_ == r.t_ && key.trip_from_ == r.stop_range_.from_ &&
          std::string_view{time} < "1650") {
        // Never jumps back to the bus behind.
        EXPECT_GE(first, last_first_covered);
        last_first_covered = first;
      }
    }
    if (std::string_view{time} >= "1650") {
      // BH411 has left the feeds: PR773 runs …075 alone, without a copy.
      ASSERT_EQ(1U, updates.size());
      EXPECT_EQ("PR773", vehicle(updates[0]));
      EXPECT_FALSE(t.resolve(c, kCopy075).valid());
    }
    previous = std::move(c);
  }
}
