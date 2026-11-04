#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "boost/json/object.hpp"

#include "geo/latlng.h"

#include "motis/rt/vehicle_position.h"

// Experimental live vehicle stream (WebSocket).
//
// A client subscribes to a viewport and gets one full snapshot, then one delta
// per published realtime cycle. Deltas are keyed by `feedId:entityId` and hold
// new vehicles in full, top-level field patches for changed vehicles and the
// keys of vehicles that left the viewport or went stale. A new subscription on
// the same connection is answered with a delta against what the connection has
// already received, so panning never resends unchanged vehicles.
namespace motis::vehicle_stream {

// One vehicle of a published realtime snapshot, serialized once per cycle and
// shared by every connection.
struct entry {
  std::string key_;
  vehicle_positions::vehicle_position const* vehicle_{nullptr};
  bool matched_{false};
  std::shared_ptr<boost::json::object const> json_;
};

struct cycle {
  // Owns what `entry::vehicle_` points to.
  std::shared_ptr<void const> keep_alive_;
  std::vector<entry> entries_;
  std::uint64_t id_{0U};
  std::int64_t published_at_ms_{0};
};

struct subscription {
  std::string id_;
  geo::latlng min_{};
  geo::latlng max_{};
  std::optional<std::vector<std::string>> language_{};
  std::optional<std::int64_t> max_age_{};
  bool include_unmatched_{false};
  // Drop what the connection has and start over with a snapshot.
  bool resync_{false};
};

using parse_result = std::variant<subscription, std::string>;

// Parses `{"type":"subscribe",...}` (minus the type check). Returns the error
// message for invalid input.
parse_result parse_subscription(boost::json::object const&);

// Parses the subscription a client may put in the connection URL query.
std::optional<parse_result> parse_subscription_query(std::string_view target);

std::string lang_key(std::optional<std::vector<std::string>> const&);

struct connection_state {
  std::optional<subscription> sub_;
  std::uint64_t seq_{0U};
  bool snapshot_sent_{false};
  std::unordered_map<std::string, std::shared_ptr<boost::json::object const>>
      sent_;
};

struct message {
  std::string json_;
  bool snapshot_{false};
  std::size_t upserts_{0U};
  std::size_t patches_{0U};
  std::size_t removes_{0U};
};

struct limits {
  std::int64_t default_max_age_{60};
  unsigned update_interval_{60U};
  std::size_t max_vehicles_{10'000U};
};

// The vehicles of `c` the subscription selects at `now` (unix seconds).
std::vector<entry const*> select_vehicles(subscription const&,
                                 cycle const&,
                                 std::int64_t now,
                                 limits const&);

// Builds the next message for the connection (a snapshot if it has not
// received one yet, a delta otherwise) and records it as sent.
// Requires a subscription.
message next_message(connection_state&,
                     cycle const&,
                     std::int64_t now,
                     limits const&);

}  // namespace motis::vehicle_stream
