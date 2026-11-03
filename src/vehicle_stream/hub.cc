#include "motis/vehicle_stream/hub.h"

#include <atomic>
#include <utility>

#include "boost/asio/post.hpp"
#include "boost/iostreams/device/back_inserter.hpp"
#include "boost/iostreams/filter/gzip.hpp"
#include "boost/iostreams/filtering_stream.hpp"
#include "boost/json.hpp"
#include "boost/url/parse.hpp"

#include "utl/logging.h"

#include "motis/config.h"
#include "motis/data.h"
#include "motis/rt/vehicle_matching.h"
#include "motis/rt/vehicle_position.h"

namespace json = boost::json;
namespace asio = boost::asio;

namespace motis::vehicle_stream {

namespace {

std::string gzip(std::string const& in) {
  auto out = std::string{};
  {
    auto os = boost::iostreams::filtering_ostream{};
    os.push(boost::iostreams::gzip_compressor{});
    os.push(boost::iostreams::back_inserter(out));
    os.write(in.data(), static_cast<std::streamsize>(in.size()));
  }
  return out;
}

std::optional<boost::urls::url_view> parse_target(std::string_view target) {
  auto const url = boost::urls::parse_origin_form(target);
  if (!url.has_value()) {
    return std::nullopt;
  }
  return *url;
}

bool wants_deflate(std::string_view const target) {
  auto const url = parse_target(target);
  if (!url.has_value()) {
    return false;
  }
  auto const params = url->params();
  auto const it = params.find("encoding");
  return it != params.end() && (*it).has_value && (*it).value == "gzip";
}

std::int64_t system_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string control(json::object o) { return json::serialize(o); }

}  // namespace

hub::hub(asio::io_context& ioc, source src, settings s)
    : strand_{asio::make_strand(ioc)},
      timer_{strand_},
      source_{std::move(src)},
      settings_{std::move(s)} {}

std::shared_ptr<hub> hub::create(asio::io_context& ioc,
                                 data& d,
                                 config const& c) {
  auto const update_interval =
      c.timetable_.has_value() ? c.timetable_->update_interval_ : 60U;
  auto s = settings{};
  s.limits_.update_interval_ = update_interval;
  s.limits_.default_max_age_ =
      vehicle_matching::default_max_age(update_interval);

  auto src = source{
      .current_ = [&d]() -> std::shared_ptr<void const> {
        return std::atomic_load(&d.rt_);
      },
      .build_ =
          [&d](std::shared_ptr<void const> const& current,
               std::optional<std::vector<std::string>> const& language) {
            auto const rt = std::static_pointer_cast<motis::rt const>(current);
            auto c = cycle{.keep_alive_ = current};
            if (rt->vehicle_positions_ == nullptr) {
              return c;
            }
            auto const& all = rt->vehicle_positions_->all();
            c.entries_.reserve(all.size());
            for (auto const& vehicle : all) {
              auto details = vehicle_matching::resolve_details(
                  d.tags_.get(), d.tt_.get(), rt->rtt_.get(), d.shapes_.get(),
                  vehicle, language);
              auto const matched =
                  details.match_state_ != api::VehicleMatchStateEnum::UNMATCHED;
              auto v = json::value_from(
                  vehicle_matching::to_api(vehicle, std::move(details), false));
              c.entries_.push_back(entry{
                  .key_ = vehicle.feed_id_ + ":" + vehicle.entity_id_,
                  .vehicle_ = &vehicle,
                  .matched_ = matched,
                  .json_ = std::make_shared<json::object const>(
                      std::move(v.as_object()))});
            }
            return c;
          }};

  return std::make_shared<hub>(ioc, std::move(src), std::move(s));
}

void hub::start() {
  asio::post(strand_, [self = shared_from_this()]() { self->schedule(); });
}

void hub::stop() {
  asio::post(strand_, [self = shared_from_this()]() {
    self->stopped_ = true;
    self->timer_.cancel();
    self->connections_.clear();
    self->by_session_.clear();
    self->cycles_.clear();
    self->n_connections_ = 0U;
  });
}

bool hub::accepts_upgrade(std::string_view const target) const {
  auto const url = parse_target(target);
  if (!url.has_value() || url->path() != kPath) {
    return false;
  }
  if (n_connections_.load() >= settings_.max_connections_) {
    // Only counted; the connection is refused with 404 by net.
    n_rejected_.fetch_add(1U);
    return false;
  }
  return true;
}

void hub::on_open(net::ws_session_ptr const& ws, std::string const& target) {
  asio::post(strand_, [self = shared_from_this(), ws, target]() {
    self->find_or_add(ws, &target);
  });
}

void hub::on_msg(net::ws_session_ptr const& ws,
                 std::string const& msg,
                 net::ws_msg_type) {
  // Checked before queueing, so a flooding peer cannot grow the strand queue.
  if (n_pending_messages_.fetch_add(1U) >= settings_.max_pending_messages_) {
    n_pending_messages_.fetch_sub(1U);
    n_dropped_messages_.fetch_add(1U);
    return;
  }
  auto const too_large = msg.size() > settings_.max_message_size_;
  asio::post(strand_, [self = shared_from_this(), ws,
                       payload = too_large ? std::string{} : msg, too_large]() {
    self->n_pending_messages_.fetch_sub(1U);
    self->handle_msg(ws, payload, too_large);
  });
}

void hub::poll_now() {
  asio::post(strand_, [self = shared_from_this()]() { self->poll(); });
}

hub::connection* hub::find_or_add(net::ws_session_ptr const& ws,
                                  std::string const* target) {
  if (stopped_) {
    return nullptr;
  }
  auto const session = ws.lock();
  if (session == nullptr) {
    return nullptr;
  }
  if (auto const it = by_session_.find(session.get());
      it != end(by_session_)) {
    return &connections_.at(it->second);
  }

  if (connections_.size() >= settings_.max_connections_) {
    // More upgrades passed accepts_upgrade concurrently than the cap allows.
    n_rejected_.fetch_add(1U);
    return nullptr;
  }

  auto const id = next_id_++;
  auto& c = connections_[id];
  c.id_ = id;
  c.ws_ = ws;
  c.deflate_ = target != nullptr && wants_deflate(*target);
  c.last_send_ = std::chrono::steady_clock::now();
  by_session_.emplace(session.get(), id);
  n_connections_ = connections_.size();

  // Safe: the session cannot be destroyed while `session` is held.
  session->on_close([w = weak_from_this(), id]() {
    if (auto const self = w.lock(); self != nullptr) {
      asio::post(self->strand_, [self, id]() { self->remove(id); });
    }
  });

  if (target != nullptr) {
    if (auto sub = parse_subscription_query(*target); sub.has_value()) {
      subscribe(c, std::move(*sub));
    }
  }
  return &c;
}

void hub::handle_msg(net::ws_session_ptr const& ws,
                     std::string const& msg,
                     bool const too_large) {
  auto* c = find_or_add(ws, nullptr);
  if (c == nullptr) {
    return;
  }

  auto const error = [&](std::string_view message) {
    c->control_ = control({{"type", "error"}, {"message", message}});
    pump(*c);
  };

  if (too_large) {
    return error("message too large");
  }
  auto ec = boost::system::error_code{};
  auto const parsed = json::parse(msg, ec);
  if (ec || !parsed.is_object()) {
    return error("expected a JSON object");
  }
  auto const& o = parsed.as_object();
  auto const type = o.find("type");
  if (type == o.end() || !type->value().is_string()) {
    return error("missing type");
  }
  auto const& t = type->value().as_string();
  if (t == "subscribe") {
    subscribe(*c, parse_subscription(o));
  } else if (t == "ping") {
    c->control_ = control({{"type", "pong"}, {"serverTime", now() * 1000}});
    pump(*c);
  } else {
    error("unknown type");
  }
}

void hub::subscribe(connection& c, parse_result&& result) {
  if (auto const* message = std::get_if<std::string>(&result)) {
    c.control_ = control({{"type", "error"}, {"message", *message}});
    pump(c);
    return;
  }
  auto sub = std::get<subscription>(std::move(result));
  if (!cycles_.contains(lang_key(sub.language_)) &&
      cycles_.size() >= settings_.max_languages_) {
    c.control_ = control({{"type", "error"},
                          {"sub", sub.id_},
                          {"message", "too many languages in use"}});
    pump(c);
    return;
  }
  c.state_.sub_ = std::move(sub);
  c.dirty_ = true;
  pump(c);
}

void hub::remove(std::uint64_t const id) {
  auto const it = connections_.find(id);
  if (it == end(connections_)) {
    return;
  }
  std::erase_if(by_session_,
                [&](auto const& entry) { return entry.second == id; });
  connections_.erase(it);
  n_connections_ = connections_.size();
}

void hub::schedule() {
  if (stopped_) {
    return;
  }
  timer_.expires_after(settings_.poll_interval_);
  timer_.async_wait([w = weak_from_this()](boost::system::error_code ec) {
    auto const self = w.lock();
    if (ec || self == nullptr || self->stopped_) {
      return;
    }
    self->poll();
    self->schedule();
  });
}

void hub::poll() {
  if (stopped_) {
    return;
  }
  auto const current = source_.current_();
  if (current != nullptr &&
      (current_.owner_before(current) || current.owner_before(current_))) {
    current_ = current;
    ++cycle_id_;
    published_at_ms_ = settings_.now_ ? now() * 1000 : system_now_ms();
    last_published_at_ms_ = published_at_ms_;
    cycles_.clear();
    n_cycles_.fetch_add(1U);
    last_tick_ = std::chrono::steady_clock::now();
    refresh_all();
  }

  auto const steady_now = std::chrono::steady_clock::now();
  // Without a new cycle (feeds down, RT update failing), still drop vehicles
  // that went stale once per update interval, as a poll would.
  if (current_.lock() != nullptr &&
      steady_now - last_tick_ >=
          std::chrono::seconds{settings_.limits_.update_interval_}) {
    last_tick_ = steady_now;
    n_ticks_.fetch_add(1U);
    refresh_all();
  }

  for (auto& [id, c] : connections_) {
    if (!c.in_flight_ && !c.control_.has_value() &&
        steady_now - c.last_send_ > settings_.heartbeat_interval_) {
      c.control_ =
          control({{"type", "heartbeat"}, {"serverTime", now() * 1000}});
      n_heartbeats_.fetch_add(1U);
      pump(c);
    }
  }
}

void hub::refresh_all() {
  for (auto& [id, c] : connections_) {
    if (!c.state_.sub_.has_value()) {
      continue;
    }
    if (c.in_flight_ && c.dirty_) {
      n_coalesced_.fetch_add(1U);
    }
    c.dirty_ = true;
    pump(c);
  }
}

cycle const* hub::cycle_for(subscription const& sub) {
  auto const key = lang_key(sub.language_);
  if (auto const it = cycles_.find(key); it != end(cycles_)) {
    return &it->second;
  }
  auto const current = current_.lock();
  if (current == nullptr || cycles_.size() >= settings_.max_languages_) {
    return nullptr;
  }
  auto const started = std::chrono::steady_clock::now();
  auto c = source_.build_(current, sub.language_);
  last_build_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - started)
                       .count();
  c.id_ = cycle_id_;
  c.published_at_ms_ = published_at_ms_;
  return &cycles_.emplace(key, std::move(c)).first->second;
}

void hub::pump(connection& c) {
  if (c.in_flight_) {
    return;
  }
  if (c.control_.has_value()) {
    auto payload = std::move(*c.control_);
    c.control_.reset();
    send(c, std::move(payload), false);
    return;
  }
  if (!c.dirty_ || !c.state_.sub_.has_value()) {
    return;
  }
  auto const* cy = cycle_for(*c.state_.sub_);
  if (cy == nullptr) {
    if (current_.lock() != nullptr) {
      // Refused: this cycle already holds as many languages as allowed.
      c.dirty_ = false;
      auto const sub_id = c.state_.sub_->id_;
      c.state_.sub_.reset();
      send(c,
           control({{"type", "error"},
                    {"sub", sub_id},
                    {"message", "too many languages in use"}}),
           false);
    }
    return;
  }
  c.dirty_ = false;
  auto msg = next_message(c.state_, *cy, now(), settings_.limits_);
  (msg.snapshot_ ? n_snapshots_ : n_deltas_).fetch_add(1U);
  send(c, std::move(msg.json_), c.deflate_);
}

void hub::send(connection& c, std::string payload, bool const binary) {
  auto const session = c.ws_.lock();
  if (session == nullptr) {
    return;
  }
  n_raw_bytes_.fetch_add(payload.size());
  if (binary) {
    payload = gzip(payload);
  }
  n_bytes_.fetch_add(payload.size());
  c.in_flight_ = true;
  c.last_send_ = std::chrono::steady_clock::now();
  session->send(
      std::move(payload),
      binary ? net::ws_msg_type::BINARY : net::ws_msg_type::TEXT,
      [w = weak_from_this(), id = c.id_](boost::system::error_code ec,
                                         std::size_t n) {
        if (auto const self = w.lock(); self != nullptr) {
          asio::post(self->strand_,
                     [self, id, ec, n]() { self->on_sent(id, ec, n); });
        }
      });
}

void hub::on_sent(std::uint64_t const id,
                  boost::system::error_code const ec,
                  std::size_t) {
  auto const it = connections_.find(id);
  if (it == end(connections_)) {
    return;
  }
  auto& c = it->second;
  c.in_flight_ = false;
  if (ec) {
    // The session closes itself; stop computing for it until it does.
    c.state_.sub_.reset();
    c.dirty_ = false;
    return;
  }
  pump(c);
}

std::int64_t hub::now() const {
  if (settings_.now_) {
    return settings_.now_();
  }
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

json::object hub::stats() const {
  return {{"connections", n_connections_.load()},
          {"cycles", n_cycles_.load()},
          {"snapshots", n_snapshots_.load()},
          {"deltas", n_deltas_.load()},
          {"heartbeats", n_heartbeats_.load()},
          {"coalesced", n_coalesced_.load()},
          {"rejected", n_rejected_.load()},
          {"ticks", n_ticks_.load()},
          {"droppedMessages", n_dropped_messages_.load()},
          {"bytesSent", n_bytes_.load()},
          {"rawBytes", n_raw_bytes_.load()},
          {"lastPublishedAt", last_published_at_ms_.load()},
          {"lastBuildMicros", last_build_us_.load()}};
}

}  // namespace motis::vehicle_stream
