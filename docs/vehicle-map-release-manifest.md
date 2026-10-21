# Vehicle Map Release Manifest

This branch reconstructs the live vehicle map feature on `fork/patches` at
`32ae30299`. The original feature history diverged from the fork at
`a0dcd8b37`; the behavior is intentionally carried as a semantic replacement
commit rather than replaying the 48 divergent commits.

| Behavior | Original commits | Replacement | Verification |
| --- | --- | --- | --- |
| Last-good GTFS-RT ingestion and mixed-source isolation | `288d596e0..8cd62dfee` | `96ddefc feat(rt): add GPS-derived vehicle predictions` | `motis_rt_update.*` in `test/rt_update_last_good_test.cc` |
| Vehicle ETA configuration, bounded history, and identity-safe snapshots | `a8ebb8a76..6a33ee8eb` | `96ddefc feat(rt): add GPS-derived vehicle predictions` | `motis.vehicle_eta*`, `vehicle_observation_history.*` |
| Shape progress projection used by matching | `ead0e49b9..42b0190fa` | `96ddefc feat(rt): add GPS-derived vehicle predictions` | `trip_progress_projection.*` |
| Vehicle snapshot API and display payload | `a754c0906`, `06d08b987`, `6f977bd03` | `62561b2 feat(api): expose live vehicle prediction data` | `vehicle_positions.*`, endpoint contract generation |
| Vehicle-only and differential realtime updates | `4f352b638`, `ab1423cf1` | `96ddefc feat(rt): add GPS-derived vehicle predictions` | `motis_rt_update.*vehicle*`, `vehicle_positions.*` |
| Static trip/route resolution and unresolved filtering | `f0a07ba48`, `e359156ec`, `1b75e9add`, `db2ba644f` | `62561b2 feat(api): expose live vehicle prediction data` | vehicle map and matching tests |
| Matching state and report-time freshness | `1726f4cbc`, `47a97086b`, `3d44b4b7a` | `62561b2 feat(api): expose live vehicle prediction data` | `vehicle_positions.*`, Trip Details tests |
| Primary vehicle selection for Trip Details | `b5fdf3710`, `ff883d39b` | `62561b2 feat(api): expose live vehicle prediction data` | `motis.trip*`, `test/endpoints/trip_test.cc` |

Operational documentation and bounded cycle work are carried by `f258161 docs(rt): document live vehicle rollout`
and `7a4b177 perf(rt): bound realtime cycle work`.

The reconstruction preserves the newer fork behavior added after the common
base, especially canonical stop coalescing, adjacent service-day GTFS-RT
resolution, and Warsaw trip-id rewriting in the collect/parse/apply pipeline.

Vehicle ETA remains disabled when the `vehicle_eta` configuration block is
absent. The map release therefore exposes positions without changing routing
or `/api/v5/stoptimes` timing.
