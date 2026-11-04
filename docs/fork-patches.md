# Fork Patches

This fork keeps `master` close to `upstream/master` and carries deployable runtime changes on `patches`.

## Active Patches

### GTFS-RT adjacent service-day resolution

- Status: still required
- Source: `piotrski/nigiri@fix/gtfsrt-adjacent-service-day`
- Purpose: resolve trip descriptors without `start_date` against a unique adjacent service day so after-midnight trips keep realtime data; reject ambiguous matches
- Tests to run:
  - `nigiri-test --gtest_filter='rt.gtfsrt_resolve_static_trip:rt.gtfsrt_resolve_adjacent_service_day'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT block trip terminal departure

- Status: still required, even after rebasing onto upstream nigiri `18bf63c` (#419). Upstream contains the same terminal guard and `pred` initialisation, but for absolute-time updates it still propagates the clamped departure delay instead of the outgoing trip's own delay, so the order-independence fix (`1075419`) must be kept until upstream has an equivalent. When rebasing, drop only the parts upstream already has and keep `rt.block_terminal*` passing.
- Deployed to production 2026-11-01 (image `ghcr.io/piotrski/motis:e24340a5`, Coolify deployment `ks4ccgwwsg4kck4oocggco00`).
- Source: `piotrski/nigiri@fix/gtfsrt-block-terminal-departure` (on top of the adjacent service-day patch; `.pkg` pins `branch=fix/gtfsrt-adjacent-service-day`, which is fast-forwarded to the pinned commit)
- Purpose: trips of a GTFS block share the terminal stop; an incoming trip's arrival delay must not overwrite the next trip's departure there (Kraków line 70 at Pleszów showed 21:34 for a 21:39 departure). A late arrival still holds the departure, but only the outgoing trip's own delay propagates along that trip, independent of the order of the trips' updates. A later update that starts after a trip's first stop keeps that stop's existing prediction (backport of the #419 `pred` initialisation).
- Tests to run:
  - `nigiri-test --gtest_filter='rt.block_terminal*:join_split.*'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT version-switch twin trips

- Status: still required while ZTP Kraków feed A labels post-midnight trips with the next timetable version's trip_id on version-switch nights. Not upstream.
- Source: `piotrski/nigiri@fix/gtfsrt-version-switch-twin` (on top of the block terminal patch; `.pkg` pins `branch=fix/gtfsrt-adjacent-service-day`, which is fast-forwarded to the pinned commit)
- Purpose: for a TripUpdate without `start_date`, compare each realtime time with the scheduled time of its own stop (matched by stop_sequence first, then stop_id; skipped stops only advance the match; a stop_id match is not used when the stop_sequence names a later visit of a loop stop). If every time is 12 h or more off and a whole number of days off (within 3 h), use the same trip on the service day the realtime implies, or a twin trip (same route, direction, stops, relative scheduled times and local departure time) on that day, with stop_sequence values translated to the twin's numbering; if no such run exists, drop the update. Vehicles that are just very late or early keep their run. Without it, Kraków buses showed "−23 h 59 min" on 2026-11-03 00:00–04:00 and the running trips had no realtime.
- Tests to run:
  - `nigiri-test --gtest_filter='rt.gtfsrt_version_switch_twin'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT service day inference

- Status: still required
- Source: `codex/live-vehicles-gps-v6` (PR #52), `src/rt/gtfsrt_service_day.cc`
- Purpose: TripUpdate and VehiclePosition descriptors without `start_date` get the service day whose scheduled run is closest to the current time before nigiri resolves them. nigiri otherwise uses the UTC date as service date (and the adjacent-day patch above only as a fallback), which put night trips after UTC midnight on the next night's run. Descriptors whose trip id only has runs more than 20 hours away (ZTP Kraków names post-midnight runs with the next service day's trip id at schedule changes) are left as sent and logged; mapping them to the run actually operating needs a trip-equivalence rule (open).
- Tests to run:
  - `motis-test --gtest_filter='gtfsrt_service_day.*'`

### Precompute stop class masks

- Status: still required
- Source: `topic/precompute-stop-class-masks`
- Purpose: compute `Place.modes` from location-level class masks instead of root-level place masks so child/platform stops return correct modes
- Tests to run:
  - verify `/api/v1/map/stops` still returns `Place.modes`
  - verify child/platform stops keep correct modes in API responses

### Exact child stoptimes fallback

- Status: still required
- Source: `topic/exact-child-stoptimes-fallback`
- Purpose: when exact child-stop boards are empty, fall back to the parent station board and filter events back to the requested child stop
- Tests to run:
  - `motis-test` `stop_times` coverage
  - exact child-platform `stoptimes` query returns departures when the station board contains events for that child stop

### Live vehicles and GPS-derived predictions

- Status: still required
- Source: `codex/live-vehicles-gps-v6` (PR #52)
- Purpose: expose identity-safe live vehicle observations and combine bounded GPS, provider, and schedule prediction candidates
- API: v6 `trip` legs carry `arrivalPrediction` / `departurePrediction` on each stop visit (`from`, `to`, `intermediateStops`), scoped to that leg's run; the block-wide `startPrediction`, `endPrediction` and `intermediateStopEvents` stay for compatibility. `journey_to_response` takes an optional stop-event hook for this, so keep it when rebasing onto upstream.
- Per-endpoint GTFS-RT counters (`/metrics`): `motis_gtfsrt_endpoint_total_entities_total`, `motis_gtfsrt_endpoint_trip_resolve_error_total` and `motis_gtfsrt_endpoint_vehicle_entities_total`, labelled `{tag, endpoint}`. `endpoint` is the last path segment of the feed URL (e.g. `TripUpdates_A.pb`; no query, userinfo or host; `[A-Za-z0-9._-]`, at most 64 characters; the endpoint's configuration index if the path is empty, suffixed with it if two endpoints of a dataset share a name). They only count messages fetched and applied in a cycle (no failed fetches, no last-good replays), unlike the per-dataset `nigiri_gtfsrt_*{tag}` counters where vehicle position entities count as successes. Unresolved trip share of a TripUpdates endpoint: `rate(..._trip_resolve_error_total) / rate(..._total_entities_total)`; vehicle position endpoints have `vehicle_entities` equal to `total_entities`.
- Trip scope: v6 `trip` takes `withBlockLegs` (default `true`, upstream behaviour: the vehicle's whole block). `withBlockLegs=false` returns only the requested trip as one leg, and `startPrediction`, `endPrediction` and `intermediateStopEvents` then describe that trip. Only `src/endpoints/trip.cc` and the `tripV6` operation change.
- Passed stops: feeds that only describe the stops ahead of a vehicle (ZTP Kraków) used to drop passed stops back to the schedule on every rebuild. `src/rt/passed_stops.cc` freezes each passed stop event at its last published time (the observed GPS passage when GPS was selected), capped at the moment the feed first omitted it and kept before the vehicle's next event, and writes it into the provider baseline every cycle while the trip stays in the feed. Predictions for those events carry `status: PASSED`. In memory only: a restart loses frozen times of stops passed before it.
- Tests to run:
  - `motis-test --gtest_filter='motis_vehicle_positions.*:vehicle_prediction*.*:trip_progress_projection.*'`
  - `motis-test --gtest_filter='motis.stop_times:motis.trip*:motis_rt_update.*'`
  - `motis-test --gtest_filter='passed_stops.*:motis_rt_update.passed_stops*'`
  - full `motis-test`

### Live vehicle stream

- Status: fork-only (experimental endpoint)
- Source: `feat/live-vehicle-stream`, `feat/live-vehicle-stream-prod`; details in [`vehicle-stream.md`](vehicle-stream.md)
- Purpose: push live vehicle positions over a WebSocket (`/api/experimental/vehicle-stream`) instead of clients polling `map/vehicles`
- `net` is pinned to `piotrski/net@feat/ws-strand-and-message-limit`: `send` is posted onto the session strand, and `web_server::set_ws_read_message_max` bounds inbound WebSocket messages (MOTIS sets 4 KiB). Return to upstream `net` once both are merged there.
- Per-endpoint GTFS-RT `update_interval`: lets one city's feeds poll faster than the global `timetable.update_interval`. The RT loop runs at the fastest interval and replays the last good snapshot of endpoints that are not due. Upstream MOTIS has no equivalent.
- Tests to run:
  - `motis-test --gtest_filter='motis_vehicle_stream.*'`
  - `motis-test --gtest_filter='motis.config*:motis_rt_update.*'`

## Resolved Upstream

### Modes on `Place`

- Status: resolved upstream
- Previous source: `origin/feat/add-modes-to-stops`
- Reason retired: upstream now exposes `Place.modes` in `openapi.yaml` and `src/place.cc`

### Trip IDs with underscores

- Status: resolved upstream
- Previous source: `origin/fix-tripid-parsing`
- Reason retired: upstream `src/tag_lookup.cc` and `test/tag_lookup_test.cc` already support underscore-heavy trip IDs

## Fork-only Automation

- `master` keeps fork-maintenance automation such as the upstream sync reminder workflow
- `patches` keeps release automation needed to publish `ghcr.io/piotrski/motis:patches`

## Sync Checklist

1. Archive the current `origin/master`, `origin/patches`, and local runtime patch tips.
2. Refresh local `master` from `upstream/master`.
3. Rebuild `patches-next` from refreshed `master`.
4. Reapply only patches marked `still required`.
5. Validate before promoting `patches-next` to `patches`.
