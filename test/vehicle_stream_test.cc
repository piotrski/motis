#include "gtest/gtest.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "boost/asio/io_context.hpp"
#include "boost/iostreams/copy.hpp"
#include "boost/iostreams/device/array.hpp"
#include "boost/iostreams/device/back_inserter.hpp"
#include "boost/iostreams/filter/gzip.hpp"
#include "boost/iostreams/filtering_streambuf.hpp"
#include "boost/json.hpp"

#include "motis/rt/vehicle_position.h"
#include "motis/vehicle_stream/hub.h"
#include "motis/vehicle_stream/protocol.h"

using namespace motis;
using namespace motis::vehicle_stream;
namespace json = boost::json;

namespace {

constexpr auto kNow = std::int64_t{1'800'000'000};

struct fake_vehicle {
  std::string id_;
  double lat_;
  double lng_;
  std::int64_t reported_;
  bool matched_{true};
  std::string status_{"IN_TRANSIT_TO"};
};

// A published realtime snapshot: owns the vehicles its cycle points to.
struct fake_rt {
  std::vector<vehicle_positions::vehicle_position> positions_;
  std::vector<fake_vehicle> vehicles_;
};

std::shared_ptr<fake_rt> make_rt(std::vector<fake_vehicle> vehicles) {
  auto rt = std::make_shared<fake_rt>();
  rt->vehicles_ = std::move(vehicles);
  for (auto const& v : rt->vehicles_) {
    auto p = vehicle_positions::vehicle_position{};
    p.feed_id_ = "feed";
    p.entity_id_ = v.id_;
    p.reported_position_.pos_ = {v.lat_, v.lng_};
    p.reported_time_ = v.reported_;
    p.ingested_time_ = v.reported_;
    p.current_status_ = v.status_;
    rt->positions_.push_back(std::move(p));
  }
  return rt;
}

cycle build(std::shared_ptr<void const> const& current) {
  auto const rt = std::static_pointer_cast<fake_rt const>(current);
  auto c = cycle{.keep_alive_ = current};
  for (auto i = 0U; i != rt->positions_.size(); ++i) {
    auto const& p = rt->positions_[i];
    auto const& v = rt->vehicles_[i];
    c.entries_.push_back(entry{
        .key_ = p.feed_id_ + ":" + p.entity_id_,
        .vehicle_ = &p,
        .matched_ = v.matched_,
        .json_ = std::make_shared<json::object const>(json::object{
            {"feedId", p.feed_id_},
            {"entityId", p.entity_id_},
            {"reportedPosition", {{"lat", v.lat_}, {"lon", v.lng_}}},
            {"reportedTime", v.reported_},
            {"currentStatus", v.status_}})});
  }
  return c;
}

subscription krakow(std::string id = "1") {
  return std::get<subscription>(parse_subscription(json::object{
      {"id", id}, {"min", "50.0,19.8"}, {"max", "50.1,20.0"}}));
}

std::vector<std::string> keys(json::array const& a) {
  auto out = std::vector<std::string>{};
  for (auto const& v : a) {
    out.emplace_back(v.is_string()
                         ? std::string{v.as_string()}
                         : std::string{v.at("entityId").as_string()});
  }
  return out;
}

json::object next(connection_state& s, cycle const& c) {
  return json::parse(next_message(s, c, kNow, limits{}).json_).as_object();
}

}  // namespace

TEST(motis_vehicle_stream, parses_subscriptions) {
  auto const sub = parse_subscription(json::object{{"id", 7},
                                                   {"min", "50.0,19.8"},
                                                   {"max", "50.1,20.0"},
                                                   {"language", {"pl"}},
                                                   {"maxAge", 180}});
  ASSERT_TRUE(std::holds_alternative<subscription>(sub));
  auto const& s = std::get<subscription>(sub);
  EXPECT_EQ("7", s.id_);
  EXPECT_DOUBLE_EQ(50.0, s.min_.lat_);
  EXPECT_DOUBLE_EQ(20.0, s.max_.lng_);
  EXPECT_EQ(std::vector<std::string>{"pl"}, s.language_.value());
  EXPECT_EQ(180, s.max_age_.value());

  EXPECT_TRUE(std::holds_alternative<std::string>(
      parse_subscription(json::object{{"min", "50.0,19.8"}})));
  EXPECT_TRUE(std::holds_alternative<std::string>(parse_subscription(
      json::object{{"min", "nope"}, {"max", "50.1,20.0"}})));
  EXPECT_TRUE(std::holds_alternative<std::string>(parse_subscription(
      json::object{{"min", "50.0,19.8"}, {"max", "50.1,20.0"}, {"maxAge", -1}})));

  EXPECT_FALSE(parse_subscription_query(kPath).has_value());
  auto const from_url = parse_subscription_query(
      "/api/experimental/vehicle-stream?encoding=gzip&min=50.0,19.8&max=50.1,"
      "20.0&language=pl&maxAge=120&id=a");
  ASSERT_TRUE(from_url.has_value());
  ASSERT_TRUE(std::holds_alternative<subscription>(*from_url));
  EXPECT_EQ("a", std::get<subscription>(*from_url).id_);
  EXPECT_EQ(120, std::get<subscription>(*from_url).max_age_.value());
}

TEST(motis_vehicle_stream, selects_fresh_matched_vehicles_inside_the_viewport) {
  auto const rt = make_rt({
      {"inside", 50.05, 19.9, kNow - 10},
      {"outside", 50.2, 19.9, kNow - 10},
      {"stale", 50.05, 19.9, kNow - 500},
      {"unmatched", 50.05, 19.9, kNow - 10, false},
  });
  auto const c = build(rt);
  auto sub = krakow();

  auto const selected = select_vehicles(sub, c, kNow, limits{.default_max_age_ = 60});
  ASSERT_EQ(1U, selected.size());
  EXPECT_EQ("feed:inside", selected[0]->key_);

  sub.include_unmatched_ = true;
  sub.max_age_ = 1000;
  EXPECT_EQ(3U, select_vehicles(sub, c, kNow, limits{}).size());
}

TEST(motis_vehicle_stream, sends_a_snapshot_then_field_deltas) {
  auto s = connection_state{.sub_ = krakow()};

  auto const first = make_rt({{"a", 50.05, 19.9, kNow - 10},
                              {"b", 50.06, 19.9, kNow - 10},
                              {"c", 50.07, 19.9, kNow - 10}});
  auto const snapshot = next(s, build(first));
  EXPECT_EQ("snapshot", snapshot.at("type").as_string());
  EXPECT_EQ(1, snapshot.at("seq").as_int64());
  EXPECT_EQ((std::vector<std::string>{"a", "b", "c"}),
            keys(snapshot.at("vehicles").as_array()));

  // a moves, b stays, c leaves, d appears.
  auto const second = make_rt({{"a", 50.051, 19.9, kNow - 5},
                               {"b", 50.06, 19.9, kNow - 10},
                               {"d", 50.08, 19.9, kNow - 5}});
  auto const delta = next(s, build(second));
  EXPECT_EQ("delta", delta.at("type").as_string());
  EXPECT_EQ(2, delta.at("seq").as_int64());
  EXPECT_EQ(std::vector<std::string>{"d"}, keys(delta.at("upserts").as_array()));
  EXPECT_EQ(std::vector<std::string>{"feed:c"},
            keys(delta.at("removes").as_array()));

  auto const& patches = delta.at("patches").as_array();
  ASSERT_EQ(1U, patches.size());
  EXPECT_EQ("feed:a", patches[0].at("key").as_string());
  auto const& set = patches[0].at("set").as_object();
  EXPECT_TRUE(set.contains("reportedPosition"));
  EXPECT_TRUE(set.contains("reportedTime"));
  EXPECT_FALSE(set.contains("feedId"));
  EXPECT_FALSE(set.contains("currentStatus"));

  // Nothing changed: an empty delta, which also serves as the cycle tick.
  auto const idle = next(s, build(second));
  EXPECT_TRUE(idle.at("upserts").as_array().empty());
  EXPECT_TRUE(idle.at("patches").as_array().empty());
  EXPECT_TRUE(idle.at("removes").as_array().empty());
}

TEST(motis_vehicle_stream, applying_deltas_reproduces_the_snapshot) {
  auto s = connection_state{.sub_ = krakow()};
  auto client = std::map<std::string, json::object>{};
  auto const apply = [&](json::object const& m) {
    if (m.at("type") == "snapshot") {
      client.clear();
      for (auto const& v : m.at("vehicles").as_array()) {
        client["feed:" + std::string{v.at("entityId").as_string()}] =
            v.as_object();
      }
      return;
    }
    for (auto const& v : m.at("upserts").as_array()) {
      client["feed:" + std::string{v.at("entityId").as_string()}] =
          v.as_object();
    }
    for (auto const& p : m.at("patches").as_array()) {
      auto& target = client.at(std::string{p.at("key").as_string()});
      for (auto const& [k, v] : p.at("set").as_object()) {
        target[k] = v;
      }
      if (auto const unset = p.as_object().find("unset");
          unset != p.as_object().end()) {
        for (auto const& k : unset->value().as_array()) {
          target.erase(k.as_string());
        }
      }
    }
    for (auto const& k : m.at("removes").as_array()) {
      client.erase(std::string{k.as_string()});
    }
  };

  auto cycles = std::vector<std::shared_ptr<fake_rt>>{
      make_rt({{"a", 50.05, 19.9, kNow - 30}, {"b", 50.06, 19.9, kNow - 30}}),
      make_rt({{"a", 50.052, 19.91, kNow - 20, true, "STOPPED_AT"},
               {"c", 50.07, 19.9, kNow - 20}}),
      make_rt({{"b", 50.06, 19.9, kNow - 10}, {"c", 50.071, 19.9, kNow - 5}}),
  };
  for (auto const& rt : cycles) {
    auto const c = build(rt);
    apply(next(s, c));

    auto expected = std::map<std::string, json::object>{};
    for (auto const* e : select_vehicles(*s.sub_, c, kNow, limits{})) {
      expected[e->key_] = *e->json_;
    }
    EXPECT_EQ(expected, client);
  }
}

TEST(motis_vehicle_stream, resubscribing_sends_only_what_changed) {
  auto s = connection_state{.sub_ = krakow("1")};
  auto const c = build(make_rt({{"west", 50.05, 19.85, kNow - 10},
                                {"both", 50.05, 19.95, kNow - 10},
                                {"east", 50.05, 20.05, kNow - 10}}));
  s.sub_->max_ = {50.1, 20.0};
  EXPECT_EQ((std::vector<std::string>{"west", "both"}),
            keys(next(s, c).at("vehicles").as_array()));

  // Pan east.
  s.sub_ = std::get<subscription>(parse_subscription(json::object{
      {"id", "2"}, {"min", "50.0,19.9"}, {"max", "50.1,20.1"}}));
  auto const pan = next(s, c);
  EXPECT_EQ("delta", pan.at("type").as_string());
  EXPECT_EQ("2", pan.at("sub").as_string());
  EXPECT_EQ(std::vector<std::string>{"east"}, keys(pan.at("upserts").as_array()));
  EXPECT_EQ(std::vector<std::string>{"feed:west"},
            keys(pan.at("removes").as_array()));
  EXPECT_TRUE(pan.at("patches").as_array().empty());

  // A resync starts over with a snapshot.
  s.sub_->resync_ = true;
  auto const resync = next(s, c);
  EXPECT_EQ("snapshot", resync.at("type").as_string());
  EXPECT_EQ((std::vector<std::string>{"both", "east"}),
            keys(resync.at("vehicles").as_array()));
}

TEST(motis_vehicle_stream, refuses_oversized_viewports) {
  auto s = connection_state{.sub_ = krakow()};
  auto const c = build(make_rt(
      {{"a", 50.05, 19.9, kNow - 10}, {"b", 50.06, 19.9, kNow - 10}}));
  auto const m = json::parse(
      next_message(s, c, kNow, limits{.max_vehicles_ = 1U}).json_);
  EXPECT_EQ("error", m.at("type").as_string());
  EXPECT_EQ(0U, s.seq_);
}

// ---------------------------------------------------------------------------
// hub

namespace {

struct fake_session : public net::ws_session {
  struct sent {
    std::string msg_;
    net::ws_msg_type type_;
    send_cb_t cb_;
  };

  ~fake_session() {
    if (on_close_) {
      on_close_();
    }
  }

  void send(std::string msg, net::ws_msg_type type, send_cb_t cb) override {
    sent_.push_back({std::move(msg), type, std::move(cb)});
  }
  void on_msg(std::function<void(std::string const&, net::ws_msg_type)>&&)
      override {}
  void on_close(std::function<void()>&& fn) override {
    on_close_ = std::move(fn);
  }

  // Completes the oldest pending write, like a client reading it.
  std::string complete() {
    auto s = std::move(sent_.at(completed_));
    ++completed_;
    s.cb_({}, s.msg_.size());
    return s.msg_;
  }

  std::size_t pending() const { return sent_.size() - completed_; }

  std::vector<sent> sent_;
  std::size_t completed_{0U};
  std::function<void()> on_close_;
};

std::string gunzip(std::string const& in) {
  auto out = std::string{};
  auto src = boost::iostreams::array_source{in.data(), in.size()};
  auto is = boost::iostreams::filtering_istreambuf{};
  is.push(boost::iostreams::gzip_decompressor{});
  is.push(src);
  boost::iostreams::copy(is, boost::iostreams::back_inserter(out));
  return out;
}

struct hub_fixture {
  explicit hub_fixture(settings s = {})
      : hub_{std::make_shared<hub>(
            ioc_,
            source{.current_ = [this]() -> std::shared_ptr<void const> {
                     return rt_;
                   },
                   .build_ =
                       [this](std::shared_ptr<void const> const& current,
                              auto const&) {
                         ++builds_;
                         return build(current);
                       }},
            [&] {
              s.heartbeat_interval_ = std::chrono::hours{1};
              s.now_ = [this] { return now_; };
              return s;
            }())} {}

  void run() {
    ioc_.restart();
    ioc_.poll();
  }

  void publish(std::vector<fake_vehicle> vehicles) {
    rt_ = make_rt(std::move(vehicles));
    hub_->poll_now();
    run();
  }

  std::shared_ptr<fake_session> open(std::string const& target) {
    auto session = std::make_shared<fake_session>();
    hub_->on_open(session, target);
    run();
    return session;
  }

  json::object last(fake_session& s) {
    return json::parse(s.sent_.back().msg_).as_object();
  }

  boost::asio::io_context ioc_;
  std::int64_t now_{kNow};
  std::shared_ptr<fake_rt> rt_;
  int builds_{0};
  std::shared_ptr<hub> hub_;
};

std::string const kKrakow =
    "/api/experimental/vehicle-stream?min=50.0,19.8&max=50.1,20.0&id=1";

}  // namespace

TEST(motis_vehicle_stream, hub_accepts_only_the_stream_path) {
  auto f = hub_fixture{};
  EXPECT_TRUE(f.hub_->accepts_upgrade(kPath));
  EXPECT_TRUE(f.hub_->accepts_upgrade(kKrakow));
  EXPECT_FALSE(f.hub_->accepts_upgrade("/api/v1/map/vehicles"));
  EXPECT_FALSE(f.hub_->accepts_upgrade("/"));
}

TEST(motis_vehicle_stream, hub_pushes_each_published_cycle) {
  auto f = hub_fixture{};
  f.publish({{"a", 50.05, 19.9, kNow - 10}});

  auto const session = f.open(kKrakow);
  ASSERT_EQ(1U, session->pending());
  EXPECT_EQ("snapshot", f.last(*session).at("type").as_string());
  session->complete();
  f.run();

  // Polling the same snapshot again pushes nothing.
  f.hub_->poll_now();
  f.run();
  EXPECT_EQ(0U, session->pending());

  f.publish({{"a", 50.051, 19.9, kNow - 5}});
  ASSERT_EQ(1U, session->pending());
  auto const delta = f.last(*session);
  EXPECT_EQ("delta", delta.at("type").as_string());
  EXPECT_EQ(1U, delta.at("patches").as_array().size());
  EXPECT_EQ(2, delta.at("cycle").as_int64());
}

TEST(motis_vehicle_stream, hub_coalesces_cycles_for_a_slow_client) {
  auto f = hub_fixture{};
  f.publish({{"a", 50.05, 19.9, kNow - 30}});
  auto const session = f.open(kKrakow);
  ASSERT_EQ(1U, session->pending());

  // The client does not read the snapshot while three cycles are published.
  f.publish({{"a", 50.051, 19.9, kNow - 20}});
  f.publish({{"a", 50.052, 19.9, kNow - 10}, {"b", 50.06, 19.9, kNow - 10}});
  f.publish({{"a", 50.053, 19.9, kNow - 5}, {"b", 50.06, 19.9, kNow - 10}});
  EXPECT_EQ(1U, session->pending());
  EXPECT_EQ(2U, f.hub_->stats().at("coalesced").as_uint64());

  // Once it catches up it gets one delta against what it actually received.
  session->complete();
  f.run();
  ASSERT_EQ(1U, session->pending());
  auto const delta = f.last(*session);
  EXPECT_EQ(4, delta.at("cycle").as_int64());
  EXPECT_EQ(std::vector<std::string>{"b"}, keys(delta.at("upserts").as_array()));
  ASSERT_EQ(1U, delta.at("patches").as_array().size());
  EXPECT_DOUBLE_EQ(50.053, delta.at("patches")
                               .as_array()[0]
                               .at("set")
                               .at("reportedPosition")
                               .at("lat")
                               .as_double());
}

TEST(motis_vehicle_stream, hub_answers_resubscribes_and_errors) {
  auto f = hub_fixture{};
  f.publish({{"a", 50.05, 19.85, kNow - 10}, {"b", 50.05, 20.05, kNow - 10}});
  auto const session = f.open(std::string{kPath});
  EXPECT_EQ(0U, session->pending());

  f.hub_->on_msg(session, R"({"type":"subscribe","id":"1","min":"50.0,19.8","max":"50.1,20.0"})",
                 net::ws_msg_type::TEXT);
  f.run();
  EXPECT_EQ("snapshot", f.last(*session).at("type").as_string());
  session->complete();
  f.run();

  f.hub_->on_msg(session, R"({"type":"subscribe","id":"2","min":"50.0,19.9","max":"50.1,20.1"})",
                 net::ws_msg_type::TEXT);
  f.run();
  auto const pan = f.last(*session);
  EXPECT_EQ("2", pan.at("sub").as_string());
  EXPECT_EQ(std::vector<std::string>{"b"}, keys(pan.at("upserts").as_array()));
  session->complete();
  f.run();

  f.hub_->on_msg(session, "not json", net::ws_msg_type::TEXT);
  f.run();
  EXPECT_EQ("error", f.last(*session).at("type").as_string());
  session->complete();
  f.run();

  f.hub_->on_msg(session, R"({"type":"ping"})", net::ws_msg_type::TEXT);
  f.run();
  EXPECT_EQ("pong", f.last(*session).at("type").as_string());

  // Both subscriptions share one cycle build.
  EXPECT_EQ(1, f.builds_);
}

TEST(motis_vehicle_stream, hub_gzips_when_asked_and_forgets_closed_sessions) {
  auto f = hub_fixture{};
  f.publish({{"a", 50.05, 19.9, kNow - 10}});
  auto session = f.open(kKrakow + "&encoding=gzip");
  ASSERT_EQ(1U, session->pending());
  EXPECT_EQ(net::ws_msg_type::BINARY, session->sent_.back().type_);
  auto const m = json::parse(gunzip(session->sent_.back().msg_));
  EXPECT_EQ("snapshot", m.at("type").as_string());
  EXPECT_EQ(1U, f.hub_->stats().at("connections").as_uint64());

  session.reset();
  f.run();
  EXPECT_EQ(0U, f.hub_->stats().at("connections").as_uint64());
}

TEST(motis_vehicle_stream, hub_drops_stale_vehicles_when_no_cycle_is_published) {
  // update interval 0: every poll without a new cycle is a freshness tick.
  auto f = hub_fixture{settings{.limits_ = {.default_max_age_ = 60,
                                            .update_interval_ = 0U}}};
  f.publish({{"a", 50.05, 19.9, kNow - 10}, {"b", 50.06, 19.9, kNow - 50}});
  auto const session = f.open(kKrakow);
  session->complete();
  f.run();

  // Realtime stops publishing; 20 s later b is older than maxAge.
  f.now_ = kNow + 20;
  f.hub_->poll_now();
  f.run();
  ASSERT_EQ(1U, session->pending());
  auto const tick = f.last(*session);
  EXPECT_EQ("delta", tick.at("type").as_string());
  EXPECT_EQ(1, tick.at("cycle").as_int64());
  EXPECT_EQ(std::vector<std::string>{"feed:b"},
            keys(tick.at("removes").as_array()));
  EXPECT_LE(1U, f.hub_->stats().at("ticks").as_uint64());
}

TEST(motis_vehicle_stream, hub_bounds_languages_connections_and_inbound_messages) {
  auto f = hub_fixture{settings{.max_connections_ = 1U,
                                .max_pending_messages_ = 2U,
                                .max_languages_ = 1U}};
  f.publish({{"a", 50.05, 19.9, kNow - 10}});

  auto const first = f.open(kKrakow + "&language=pl");
  EXPECT_EQ("snapshot", f.last(*first).at("type").as_string());
  first->complete();
  f.run();

  // A second language would need a second serialization of the cycle.
  f.hub_->on_msg(first,
                 R"({"type":"subscribe","id":"2","min":"50.0,19.8","max":"50.1,20.0","language":["de"]})",
                 net::ws_msg_type::TEXT);
  f.run();
  EXPECT_EQ("error", f.last(*first).at("type").as_string());
  EXPECT_EQ(1, f.builds_);
  first->complete();
  f.run();

  // Over the connection cap: never registered, never sent anything.
  auto const second = f.open(kKrakow);
  EXPECT_EQ(0U, second->pending());
  EXPECT_EQ(1U, f.hub_->stats().at("connections").as_uint64());
  EXPECT_EQ(1U, f.hub_->stats().at("rejected").as_uint64());

  // Messages beyond the pending cap are dropped before they are queued.
  for (auto i = 0; i != 5; ++i) {
    f.hub_->on_msg(first, R"({"type":"ping"})", net::ws_msg_type::TEXT);
  }
  EXPECT_EQ(3U, f.hub_->stats().at("droppedMessages").as_uint64());
  f.run();
}

TEST(motis_vehicle_stream, hub_caps_languages_when_a_deferred_update_builds_its_cycle) {
  auto f = hub_fixture{settings{.max_languages_ = 1U}};
  f.publish({{"a", 50.05, 19.9, kNow - 10}});
  auto const polish = f.open(kKrakow + "&language=pl");
  ASSERT_EQ(1U, polish->pending());

  // The Polish snapshot is still being written when the next cycle arrives,
  // and a German subscriber takes the only language slot of that cycle.
  f.publish({{"a", 50.051, 19.9, kNow - 5}});
  auto const german = f.open(kKrakow + "&language=de");
  EXPECT_EQ("snapshot", f.last(*german).at("type").as_string());

  polish->complete();
  f.run();
  ASSERT_EQ(1U, polish->pending());
  EXPECT_EQ("error", f.last(*polish).at("type").as_string());
  EXPECT_EQ(2, f.builds_);
}
