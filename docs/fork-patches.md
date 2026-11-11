# Fork Patches

This fork keeps `master` close to `upstream/master` and carries deployable runtime changes on `patches`.

## Active Patches

### nigiri base

- `.pkg` pins `piotrski/nigiri@fix/gtfsrt-duplicated-trips` (`e48ce0fc`), the duplicated-trips patch below on top of `patches/upstream-1ccdf697` (`56b13d78`): upstream nigiri `1ccdf697` (#452, the same commit upstream MOTIS `4e2b1676a` uses) with the fork's GTFS-RT patches below rebased on top, plus `56b13d78`, which ports the fork's rt tests to `loader::load` after #450. The old pin was `fix/gtfsrt-adjacent-service-day` at `86e872d6`.
- Picked up with this base: update_lbs perf (`aadccd05`, #445), the bitfield copy fix (`e07e3bea`), compact timetable storage (#449 `8ab457b2`, #450 `012e623b`, #451 `49cdab21`), one trip id per trip (#452), route flags, `transport_mode_t` offsets, reflexive transfer times, and RT changes #419, #439 and #441.
- `nigiri_bin_ver` goes from 33 to 37, so every deployment needs a full re-import.
- Behavior changes picked up:
  - Wheelchair routing only uses trips whose GTFS `trips.txt` has `wheelchair_accessible=1` (nigiri `e17774c`). Feeds without that column get no wheelchair transit routes.
  - The first and last walk legs are shifted so they arrive exactly at departure (or leave at arrival), so an itinerary can start later than the query time. `journey_to_response` computes `duration` from the first and last leg times (ported from upstream MOTIS #1516).
- Not ported from upstream MOTIS: the #1593 offset redesign (the fork only aliases `transport_mode_id_t` to nigiri's `transport_mode_t::payload_t`) and #1616.
- Tests to run:
  - `nigiri-test` (full)
  - `motis-test` (full; compare against the known failures)

### GTFS-RT adjacent service-day resolution

- Status: still required
- Source: `piotrski/nigiri@fix/gtfsrt-adjacent-service-day`, rebased onto `patches/upstream-1ccdf697`, which `.pkg` pins
- Purpose: resolve trip descriptors without `start_date` against a unique adjacent service day so after-midnight trips keep realtime data; reject ambiguous matches
- Tests to run:
  - `nigiri-test --gtest_filter='rt.gtfsrt_resolve_static_trip:rt.gtfsrt_resolve_adjacent_service_day'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT block trip terminal departure

- Status: still required, reduced. Since the base includes upstream `18bf63c` (#419) with the same guard and `pred` initialisation, the patch is only `9f914290` (propagate the outgoing trip's own delay after a held departure) plus its tests. Drop it if upstream adopts the same rule; keep the order-independence tests passing.
- Source: `piotrski/nigiri@fix/gtfsrt-block-terminal-departure` (on top of the adjacent service-day patch; rebased onto `patches/upstream-1ccdf697`, which `.pkg` pins)
- Purpose: trips of a GTFS block share the terminal stop; an incoming trip's arrival delay must not overwrite the next trip's departure there (Kraków line 70 at Pleszów showed 21:34 for a 21:39 departure). A late arrival still holds the departure, but only the outgoing trip's own delay propagates along that trip, independent of the order of the trips' updates. A later update that starts after a trip's first stop keeps that stop's existing prediction (backport of the #419 `pred` initialisation).
- Tests to run:
  - `nigiri-test --gtest_filter='rt.block_terminal*:join_split.*'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT version-switch twin trips

- Status: still required while ZTP Kraków feed A labels post-midnight trips with the next timetable version's trip_id on version-switch nights. Not upstream.
- Source: `piotrski/nigiri@fix/gtfsrt-version-switch-twin` (on top of the block terminal patch; rebased onto `patches/upstream-1ccdf697`, which `.pkg` pins)
- Purpose: for a TripUpdate without `start_date`, compare each realtime time with the scheduled time of its own stop (matched by stop_sequence first, then stop_id; skipped stops only advance the match; a stop_id match is not used when the stop_sequence names a later visit of a loop stop). If every time is 12 h or more off and a whole number of days off (within 3 h), use the same trip on the service day the realtime implies, or a twin trip (same route, direction, stops, relative scheduled times and local departure time) on that day, with stop_sequence values translated to the twin's numbering; if no such run exists, drop the update. Vehicles that are just very late or early keep their run. Without it, Kraków buses showed "−23 h 59 min" on 2026-11-03 00:00–04:00 and the running trips had no realtime.
- Tests to run:
  - `nigiri-test --gtest_filter='rt.gtfsrt_version_switch_twin'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT duplicate trip split

- Status: still required while ZTP Kraków sends two TripUpdates for one trip when a second vehicle stands in. Not upstream.
- Source: `piotrski/nigiri@fix/gtfsrt-duplicated-trips` (on top of `patches/upstream-1ccdf697`, which `.pkg` pins), `src/rt/gtfsrt_duplicate_trips.cc`
- Purpose: on 2026-11-10 line 139, BH411 and PR773 both sent a TripUpdate for `20261109_1_130082075_11`. nigiri applied both to one run, last writer wins per stop, and passed stops froze BH411's times while PR773 was half an hour behind. Before nigiri sees a cycle's feeds, `split_duplicate_trip_updates` (called from `rt_update.cc`) looks at every run with more than one SCHEDULED TripUpdate. Vehicles are named by licence plate (joined from the dataset's VehiclePositions by vehicle id, since ZTP TripUpdates only carry `vehicle.id`). Of the vehicles whose GPS (at most 120 s old) is on the run, the one closest to schedule keeps the run. Each other one becomes a `DUPLICATED` TripUpdate with `trip_properties.trip_id` `<trip>~<plate>`, and its VehiclePosition is moved to that id. TripUpdates of vehicles not on the run are dropped (forecasts for the next trip, ghosts). If no vehicle is on the run yet, the fresh TripUpdate closest to schedule stays. Runs whose TripUpdates name no vehicle are left alone.
- nigiri: `DUPLICATED` with a new `trip_properties.trip_id` and no new start date or time adds an RT-only run that keeps the original's stops and static schedule (`rt_timetable::duplicated_from`), so it shows its own delay and trip name. A copy with a shifted start is counted as unsupported. `vehicle_prediction_selection` skips `DUPLICATED` updates, and `passed_stops` only follows SCHEDULED ones, so a copy never feeds the original run.
- Decision record: transit `docs/adr/0002-split-duplicate-trip-updates.md`
- Tests to run:
  - `nigiri-test --gtest_filter='duplicated_fixture.*:rt.*'`
  - `motis-test --gtest_filter='gtfsrt_duplicate_trips.*:passed_stops.*:vehicle_prediction*.*:motis_rt_update.*'`

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
