#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "boost/asio/io_context.hpp"
#include "boost/asio/steady_timer.hpp"
#include "boost/asio/strand.hpp"
#include "boost/json/object.hpp"

#include "net/web_server/web_server.h"

#include "motis/fwd.h"
#include "motis/vehicle_stream/protocol.h"

namespace motis::vehicle_stream {

constexpr auto kPath = std::string_view{"/api/experimental/vehicle-stream"};

struct source {
  // The realtime snapshot currently published. A new owner means a new cycle.
  std::function<std::shared_ptr<void const>()> current_;

  // Serializes every vehicle of the current snapshot once.
  std::function<cycle(std::shared_ptr<void const> const& current,
                      std::optional<std::vector<std::string>> const& language)>
      build_;
};

struct settings {
  std::chrono::milliseconds poll_interval_{250};
  std::chrono::seconds heartbeat_interval_{25};
  std::size_t max_connections_{1000U};
  std::size_t max_message_size_{4096U};
  // Messages received but not handled yet, over all connections.
  std::size_t max_pending_messages_{1024U};
  // Distinct languages serialized per cycle.
  std::size_t max_languages_{4U};
  limits limits_{};
  std::function<std::int64_t()> now_{};  // unix seconds, for tests
};

// Fans published realtime cycles out to WebSocket subscribers.
//
// All state lives on one strand. Each connection has at most one message in
// flight: cycles that arrive meanwhile are coalesced, and the delta is computed
// against what the connection actually received when its write completes, so a
// slow client gets fewer, larger deltas instead of an unbounded queue.
struct hub : public std::enable_shared_from_this<hub> {
  hub(boost::asio::io_context&, source, settings);

  static std::shared_ptr<hub> create(boost::asio::io_context&,
                                     data&,
                                     config const&);

  void start();
  void stop();

  // net::web_server hooks; callable from any thread.
  bool accepts_upgrade(std::string_view target) const;
  void on_open(net::ws_session_ptr const&, std::string const& target);
  void on_msg(net::ws_session_ptr const&,
              std::string const& msg,
              net::ws_msg_type);

  boost::json::object stats() const;

  // Runs one poll of the realtime source on the strand (tests).
  void poll_now();

private:
  struct connection {
    std::uint64_t id_{0U};
    net::ws_session_ptr ws_;
    connection_state state_{};
    bool deflate_{false};
    bool in_flight_{false};
    bool dirty_{false};
    std::optional<std::string> control_{};
    std::chrono::steady_clock::time_point last_send_{};
  };

  connection* find_or_add(net::ws_session_ptr const&,
                          std::string const* target);
  void handle_msg(net::ws_session_ptr const&,
                  std::string const&,
                  bool too_large);
  void subscribe(connection&, parse_result&&);
  void remove(std::uint64_t id);
  void poll();
  void refresh_all();
  void schedule();
  cycle const* cycle_for(subscription const&);
  void pump(connection&);
  void send(connection&, std::string payload, bool binary);
  void on_sent(std::uint64_t id, boost::system::error_code, std::size_t);
  std::int64_t now() const;

  boost::asio::strand<boost::asio::io_context::executor_type> strand_;
  boost::asio::steady_timer timer_;
  source source_;
  settings settings_;
  bool stopped_{false};

  std::uint64_t next_id_{1U};
  std::unordered_map<std::uint64_t, connection> connections_;
  std::unordered_map<net::ws_session const*, std::uint64_t> by_session_;

  std::weak_ptr<void const> current_;
  std::uint64_t cycle_id_{0U};
  std::chrono::steady_clock::time_point last_tick_{};
  std::int64_t published_at_ms_{0};
  std::map<std::string, cycle> cycles_;

  std::atomic_size_t n_connections_{0U};
  std::atomic_size_t n_pending_messages_{0U};
  std::atomic_uint64_t n_ticks_{0U};
  std::atomic_uint64_t n_dropped_messages_{0U};
  std::atomic_uint64_t n_cycles_{0U};
  std::atomic_uint64_t n_snapshots_{0U};
  std::atomic_uint64_t n_deltas_{0U};
  std::atomic_uint64_t n_heartbeats_{0U};
  std::atomic_uint64_t n_coalesced_{0U};
  mutable std::atomic_uint64_t n_rejected_{0U};
  std::atomic_uint64_t n_bytes_{0U};
  std::atomic_uint64_t n_raw_bytes_{0U};
  std::atomic_int64_t last_published_at_ms_{0};
  std::atomic_int64_t last_build_us_{0};
};

}  // namespace motis::vehicle_stream
