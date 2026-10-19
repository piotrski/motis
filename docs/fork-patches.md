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

### GTFS-RT service day inference

- Status: still required
- Source: `codex/live-vehicles-gps-v6` (PR #52), `src/rt/gtfsrt_service_day.cc`
- Purpose: TripUpdate and VehiclePosition descriptors without `start_date` get the service day whose scheduled run is closest to the current time before nigiri resolves them. nigiri otherwise uses the UTC date as service date (and the adjacent-day patch above only as a fallback), which put night trips after UTC midnight on the next night's run. Descriptors whose trip id only has runs more than 20 hours away (ZTP Kraków names post-midnight runs with the next service day's trip id at schedule changes) are not attached: the TripUpdate is dropped, the vehicle keeps its position without a trip.
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
- Passed stops: feeds that only describe the stops ahead of a vehicle (ZTP Kraków) used to drop passed stops back to the schedule on every rebuild. `src/rt/passed_stops.cc` freezes each passed stop event at its last published time (the observed GPS passage when GPS was selected), capped at the moment the feed first omitted it and kept before the vehicle's next event, and writes it into the provider baseline every cycle while the trip stays in the feed. Predictions for those events carry `status: PASSED`. In memory only: a restart loses frozen times of stops passed before it.
- Tests to run:
  - `motis-test --gtest_filter='motis_vehicle_positions.*:vehicle_prediction*.*:trip_progress_projection.*'`
  - `motis-test --gtest_filter='motis.stop_times:motis.trip*:motis_rt_update.*'`
  - `motis-test --gtest_filter='passed_stops.*:motis_rt_update.passed_stops*'`
  - full `motis-test`

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
