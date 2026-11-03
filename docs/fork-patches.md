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

- Status: still required. Drop it once the fork is rebased onto upstream nigiri at or after `18bf63c` (#419), which contains the same guard and `pred` initialisation; keep the order-independence tests passing when doing so.
- Source: `piotrski/nigiri@fix/gtfsrt-block-terminal-departure` (on top of the adjacent service-day patch; `.pkg` pins `branch=fix/gtfsrt-adjacent-service-day`, which is fast-forwarded to the pinned commit)
- Purpose: trips of a GTFS block share the terminal stop; an incoming trip's arrival delay must not overwrite the next trip's departure there (Kraków line 70 at Pleszów showed 21:34 for a 21:39 departure). A late arrival still holds the departure, but only the outgoing trip's own delay propagates along that trip, independent of the order of the trips' updates. A later update that starts after a trip's first stop keeps that stop's existing prediction (backport of the #419 `pred` initialisation).
- Tests to run:
  - `nigiri-test --gtest_filter='rt.block_terminal*:join_split.*'`
  - `nigiri-test --gtest_filter='rt.*'`

### GTFS-RT version-switch twin trips

- Status: still required while ZTP Kraków feed A labels post-midnight trips with the next timetable version's trip_id on version-switch nights. Not upstream.
- Source: `piotrski/nigiri@fix/gtfsrt-version-switch-twin` (on top of the block terminal patch; `.pkg` pins `branch=fix/gtfsrt-adjacent-service-day`, which is fast-forwarded to the pinned commit)
- Purpose: for a TripUpdate without `start_date`, compare each realtime time with the scheduled time of its own stop (matched by stop_sequence first, then stop_id; skipped stops only advance the match). If every time is 12 h or more off and a whole number of days off (within 3 h), use the same trip on the service day the realtime implies, or a twin trip (same route, direction, stops, relative scheduled times and local departure time) on that day, with stop_sequence values translated to the twin's numbering; if no such run exists, drop the update. Vehicles that are just very late or early keep their run. Without it, Kraków buses showed "−23 h 59 min" on 2026-11-03 00:00–04:00 and the running trips had no realtime.
- Tests to run:
  - `nigiri-test --gtest_filter='rt.gtfsrt_version_switch_twin'`
  - `nigiri-test --gtest_filter='rt.*'`

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
