# Live vehicle stream (experimental)

`ws://<host>/api/experimental/vehicle-stream[?encoding=gzip][&min=..&max=..&language=..&maxAge=..&id=..]`

Pushes the vehicles of `/api/v1/map/vehicles` for one viewport per connection, once per published realtime
cycle, instead of clients polling. Same selection as the HTTP endpoint (viewport, `maxAge` freshness, matched
vehicles unless `includeUnmatched`), without shapes.

## Messages

Client → server (text):

- `{"type":"subscribe","id":"3","min":"lat,lon","max":"lat,lon","language":["pl"],"maxAge":180}`; replaces the
  connection's subscription. `"resync":true` asks for a fresh snapshot.
- `{"type":"ping"}` → `{"type":"pong","serverTime":ms}`.

Server → client (text, or gzipped JSON in binary frames with `encoding=gzip`):

- `snapshot`: `{type, seq, sub, cycle, publishedAt, serverTime, updateInterval, vehicles: [...]}`. The first frame of a
  connection, and the answer to `resync`.
- `delta`: same header, then `upserts` (new vehicles in full), `patches` (`{key, set: {field: value}, unset: [field]}`
  with top-level fields of `VehiclePosition`) and `removes` (keys). Keys are `feedId:entityId`.
  - It covers everything since the last frame the connection got, including frames for earlier subscriptions. A new
    subscription is answered with a delta against what the connection holds, so clients apply every frame in order and
    only show frames whose `sub` is current.
  - `seq` counts snapshots and deltas per connection, and a gap means resync.
- `heartbeat` after 25 s without a frame, for proxies with idle timeouts (Cloudflare: 100 s).
- `error` (`{message, sub?}`), e.g. too many vehicles or languages.

A data frame goes out at least once per `update_interval`: for each published cycle, and without one (feeds down)
to drop vehicles that went stale.

## Server

- `motis::vehicle_stream::hub` (`src/vehicle_stream/hub.cc`) runs on one strand.
  - **Cycles:** it notices a new `data::rt_` within 250 ms (owner change, so no ABA), and serializes each vehicle
    once per cycle and language.
  - **Delivery:** one message in flight per connection. Cycles arriving meanwhile are coalesced, and the delta is
    computed against what the connection actually received.
  - **Limits:** caps connections, pending inbound messages, message size and languages per cycle.
  - **Stats:** `GET /api/experimental/vehicle-stream/stats`.
- The wiring is about 20 lines in `server.cc` and `motis_instance.h`. `rt_update.cc` only gains the per-endpoint
  `update_interval` (below).

## Per-endpoint `update_interval`

A GTFS-RT entry under `timetable.datasets.<id>.rt` can set its own poll interval in seconds:

```yaml
timetable:
  update_interval: 60
  datasets:
    pl-ZTP-Kraków-A:
      rt:
        - url: https://…/VehiclePositions_A.pb
          update_interval: 10
```

- The RT loop runs at the smallest interval configured (`timetable::rt_loop_interval()`). An endpoint is fetched when
  its own interval (or the global one) has elapsed since its last fetch, and skipped on the ticks in between.
- Every tick still rebuilds the realtime timetable from all GTFS-RT endpoints. A skipped endpoint replays its last
  good snapshot. The replay does not re-ingest vehicle positions and does not count as a successful update in metrics.
- VDV AUS and SIRI endpoints are polled on every tick. `update_interval` on them is rejected, as are `0` and values
  `>= last_good_ttl`.
- The stream's `updateInterval` is the loop period. Vehicle max age, position retention and calibration stay tied to
  the global `update_interval`, so vehicles from slow feeds do not expire between their fetches.
- Cost: each tick rebuilds the whole rtt. With 60 s globally and 10 s for one city, the rebuild CPU is six times
  higher than before. Watch `motis_rt_cycle_duration_seconds` and `motis_rt_cycle_overrun_total` after enabling it.
- `motis_rt_cycle_workload{kind="fetched_endpoints"}` shows how many endpoints were fetched in the last tick.

## `net` fork

The stream needs two changes in `net`, carried on
[piotrski/net `feat/ws-strand-and-message-limit`](https://github.com/piotrski/net/tree/feat/ws-strand-and-message-limit)
and pinned in `.pkg`:

- `net::ws_session::send` posts onto the socket's strand. Before, it touched the Beast stream on the caller's thread
  while `net` runs each socket on its own strand, so sends from the hub raced the socket's reads (pings, pongs, close).
- `web_server::set_ws_read_message_max`. `server.cc` sets it to `vehicle_stream::kMaxClientMessageSize` (4 KiB), so
  an oversized client message fails the read and the socket closes with code 1009 before the message is buffered.
  Beast's default is 16 MiB. The hub's own size check stays as a second guard.

Both are candidates for upstreaming to motis-project/net. Once upstream has them, point `.pkg` back at it.
