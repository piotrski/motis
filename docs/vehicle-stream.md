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
- `rt_update.cc` is untouched. The wiring is about 20 lines in `server.cc` and `motis_instance.h`.

## Requires a `net` fix

`net::ws_session::send` touches the Beast stream on the caller's thread, but `net` runs each socket on its own
strand, so sends from another thread race the socket's reads (pings, pongs, close). The prototype build applies
`patches/net/0001-websocket-send-on-strand.patch` to `deps/net`, which posts `send` onto the socket's executor.
To ship it, upstream that to motis-project/net (or point `.pkg` at a fork) and drop the patch.

Also needed in `net` before production: a `read_message_max` on the WebSocket stream (Beast defaults to 16 MiB), so
oversized client messages are refused before they are buffered. The hub only rejects them after the fact.
