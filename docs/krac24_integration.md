# krac24 integration: precision-lander proxy control ported into the BT

## Background

This repo and the team's GitHub `cjfgus814123/krac24` diverged from the same
pre-BT baseline (single-node `vtol_fsm.cpp` / `vtol_fsm_P.cpp` FSMs). Since
then:

- **This repo** rebuilt that FSM logic as a BehaviorTree (`krac_control/src/bt_*`,
  `mission_context`, `krac_bt_runner`).
- **krac24** kept extending the FSM node itself and reached "First Full Mission
  Test" (2026-07-03), adding a dedicated precision-landing node
  (`precision_lander.cpp`), a new vision pipeline (`vision_tracker.py` +
  `TargetError` message), a competition logger, and a trained YOLO model.

Chosen reference for the "real" control logic: krac24's `src/krac_control/src/vtol_fsm.cpp`
(despite the filename, this is the "proxy FSM" — it flies a normal
`AUTO.MISSION` plan and only intervenes with `OFFBOARD` at specific waypoint
sequences, delegating fine control to `precision_lander`, then resumes the
same plan via a forced VTOL transition). That file only covers rescue pickup
and vertiport landing; the "drop" leg does not exist there either, so it was
newly designed here reusing the pre-existing Point-based search/align/descend
pipeline.

Core architectural shift: the previous tree uploaded a **separate mission plan
per leg** and used **GPS-radius arrival detection** (`OutboundRoute` /
`ReturnMission` + `EnterREPStation` / `EnterHomeStation`). The krac24-based
design uploads **one continuous `AUTO.MISSION` plan** and uses
**`WaypointReached`-triggered intervention**, resuming via `WaypointSetCurrent`
+ a forced VTOL transition.

## New files ported from krac24

| File | Purpose |
|---|---|
| `krac_interfaces/msg/TargetError.msg` | `is_detected`, `pixel_err_x/y`, `yaw_err_rad` — precision-landing vision message |
| `krac_control/src/precision_lander.cpp` | Standalone node: PID + yaw alignment, subscribes `/vision/target_error`, publishes `/precision_lander/cmd_vel`, toggled via `/precision_lander/enable` (`std_srvs/SetBool`) |
| `krac_vision/krac_vision/vision_tracker.py` | ArUco + YOLO-OBB detector publishing `TargetError` on `/vision/target_error` (always running, no activate/deactivate step needed) |
| `krac_vision/weights/best.pt` | Trained YOLO OBB model (git-ignored via `*.pt`, ships alongside the package) |

Fixed while porting: `vision_tracker.py` had a hard-coded `/home/kch/...` model
path (same issue already tracked in `docs/known_issues.md` item 1) — now
resolves the path via `get_package_share_directory('krac_vision')` with a
`model` parameter override, same pattern as `yolo_detector.py`.

Not ported as running nodes: krac24's `offboard.cpp`, `vtol_fsm.cpp`,
`vtol_fsm_P.cpp` (the monolithic FSM binaries). Their control logic was
translated into BT nodes instead; this repo's own pre-BT `vtol_fsm.cpp` /
`vtol_fsm_P.cpp` / `transition_monitor.cpp` are left in place (still building)
but are legacy and unused by the BT flow.

## `mission_loader.py`

Merged krac24's robustness fixes into our existing loader (installed by
`krac_control`, invoked by `MissionContext::runExternalMissionLoader`):

- null-safe `param[i]` handling for the first (takeoff) waypoint
- defensive `None` check on the `WaypointPush` response before reading `.success`
- **new**: the script now exits non-zero on upload failure — previously it
  always exited 0, so `UploadMissionPlan` / `RetryUntilSuccessful` in the BT
  could never detect a real failure.

## `MissionContext` additions (`mission_context.hpp` / `.cpp`)

- `TargetError` subscription (`/vision/target_error`) with freshness tracking — separate from the existing `camera/target_error` (`geometry_msgs/Point`) used by the Point-based search/align pipeline. Both vision paths coexist.
- `mavros/mission/waypoints` (`WaypointList`) subscription, cached for heading computation.
- `lastReachedWaypointSeq()` — exposes the previously-unused `wp_reached_` subscription.
- `setMissionCurrentWaypoint` via new `WaypointSetCurrent` client (`missionSetCurrentClient()`).
- `setPrecisionLanderEnabled(bool)` — calls `/precision_lander/enable` and relays `/precision_lander/cmd_vel` straight to the existing `mavros/setpoint_velocity/cmd_vel_unstamped` publisher while enabled (mirrors krac24's `lander_vel_cb` relay pattern).
- `computeBearingToWaypoint(seq)` + `currentYaw()` — haversine bearing → ENU yaw, ported from krac24's `PHASE_ASCEND_WITH_VICTIM` auto-heading calculation.
- `publishMissionPhase(phase)` — publishes `krac_interfaces/msg/FlightPhase` on `/krac/mission_phase` (previously documented but never implemented).
- New ROS params in `krac_bt_params.yaml`: `rescue_wp_seq` (6), `resume_wp_seq` (7), `drop_wp_seq` (10), `landing_wp_seq` (13), `safe_ascend_alt` (30.0). **These are placeholders** — see Open items below.

## New BT nodes (`bt_actions_precision.hpp` / `.cpp`)

| Node | Type | Purpose |
|---|---|---|
| `IsWaypointReached` | Condition | one-shot check of `lastReachedWaypointSeq()` |
| `WaitForWaypointReached` | Stateful action | blocks (RUNNING) until a waypoint sequence is reached or timeout |
| `SetMissionCurrentWaypoint` | Stateful action | `WaypointSetCurrent` service call |
| `EnablePrecisionLander` | Sync action | toggles `precision_lander` |
| `IsPrecisionTargetDetected` | Condition | `TargetError` freshness/detection check |
| `PrecisionLandOnTarget` | Stateful action | enables lander, waits for altitude/landed, always disables lander on exit (including `onHalted`) |
| `AlignHeadingToWaypoint` | Stateful action | rotate-in-place to face a target waypoint (ported `PHASE_ALIGN_HEADING`) |
| `SetMissionPhase` | Sync action | publishes `FlightPhase` |
| `OpenGripper` | Stateful action | mirrors `CloseGripper`, used for the drop leg |

All registered in `bt_register.cpp`.

## Tree restructure (`krac_control/bt/krac_mission_bt_robust.xml`)

`EmergencyBranch` / `SafetyGuard` / `PreflightCheck` / `TakeoffPhase` kept as-is
(with `SetMissionPhase` calls added at phase boundaries). `OutboundRoute` /
`VisionBasedLanding` / `ReturnMission` (GPS-radius design) were replaced by:

1. **MissionUpload** — upload `way3.plan` once, switch `AUTO.MISSION`.
2. **CruiseToRescue** — `WaitForWaypointReached(rescue_wp_seq)`.
3. **RescueOperation** — `OFFBOARD` + `MC` transition → `EnablePrecisionLander` → `PrecisionLandOnTarget` → `CloseGripper` → `VerifyBasketPicked`.
4. **ResumeAfterRescue** — ascend to `safe_ascend_alt` → `AlignHeadingToWaypoint(resume_wp_seq)` → `SetMissionCurrentWaypoint` → `FW` transition → `AUTO.MISSION`.
5. **CruiseToDrop** — `WaitForWaypointReached(drop_wp_seq)`.
6. **DropOperation** *(new leg, no krac24 equivalent)* — reuses the existing Point-based (`yolo_detector.py`) search/align/descend pipeline, descends only to hover altitude (not a full landing) then `OpenGripper`.
7. **ResumeAfterDrop** — same pattern as step 4.
8. **CruiseToLanding** — `WaitForWaypointReached(landing_wp_seq)`.
9. **FinalLanding** — same precision-lander pattern as rescue, then `AUTO.LAND` + `DetectLanding` + `VerifyLandingComplete` + `SetMissionPhase(MISSION_COMPLETE)`.

Existing recovery nodes (`RecoverVisionLanding`, `RecoverFinalLanding`,
`GlobalMissionRecovery`) are reused unchanged in every leg.

Legacy nodes (`EnterREPStation`, `EnterHomeStation`, `IsAtREPRegion`,
`IsAtHomeRegion`, `EnsureFixedWingCruise`) are still registered/compiled but no
longer referenced by the main tree.

## Verification done

- `colcon build --packages-select krac_interfaces krac_control krac_utils krac_vision` — all succeed.
- `ros2 run krac_control krac_bt_runner` with the new XML — tree parses, all new node types register, ticks correctly through `EmergencyBranch`/`SafetyGuard` without MAVROS running.
- `ros2 run krac_control precision_lander` — starts and shuts down cleanly.

## Open items

- `ros2_ws/src/bt.xml` (repo root, untracked) looks like a stale draft of
  `krac_mission_bt_robust.xml` and isn't referenced by any launch file —
  candidate for deletion, not yet removed pending confirmation.
- `DropOperation`'s exact release behavior (hover-and-release vs. touch-and-go)
  is a first design pass, not validated against the rulebook or krac24 (which
  has no equivalent leg).
- Real drop-zone waypoint: `way3.plan` still has no drop-zone item (see
  "Waypoint seq fix" below), so `DropOperation`'s design above is untested
  against a real trajectory.

## Waypoint seq fix (resolved 2026-07-05)

`rescue_wp_seq` / `resume_wp_seq` / `drop_wp_seq` / `landing_wp_seq` were
placeholders (6 / 7 / 10 / 13) picked before `way3.plan` was finalized —
`landing_wp_seq=13` didn't even exist in the plan (only 11 items, seq 0-10),
and `rescue_wp_seq=6` pointed at `WP4_BACK`, a return-leg waypoint with no
relation to the actual rescue pickup.

Actual `way3.plan` item layout (index = MAVROS `wp_seq`, matches array order,
not `doJumpId`):

| seq | amsr_name | note |
|---|---|---|
| 0 | TAKEOFF_HOME_DIRECT_RESCUE | VTOL takeoff |
| 1 | REP_DIRECT | arrival near REP, alt 10 |
| 2 | REP_DESCEND_LOW | alt 1 (superseded by BT OFFBOARD intervention) |
| 3 | REP_HOLD_3S_LOW | alt 1, hold (superseded) |
| 4 | REP_TAKEOFF_AGAIN | alt 10 (superseded) |
| 5-9 | WP5_BACK … WP1_BACK | return leg |
| 10 | HOME_RETURN | final arrival, home/vertiport |

Decision: keep the current 11-item plan as-is and fix the BT to match it,
rather than fabricate placeholder drop-zone coordinates. Since there is no
drop-zone waypoint, the drop leg is disabled rather than pointed at a wrong
seq:

- `rescue_wp_seq: 1` — BT takes OFFBOARD control right when the mission
  reaches `REP_DIRECT`, so it does its own descent via `precision_lander`;
  the plan's own `REP_DESCEND_LOW` / `REP_HOLD_3S_LOW` / `REP_TAKEOFF_AGAIN`
  items (seq 2-4) never actually execute.
- `resume_wp_seq: 5` — after rescue, `ResumeAfterRescue` climbs to
  `safe_ascend_alt` itself and resumes `AUTO.MISSION` at `WP5_BACK`, skipping
  the now-redundant seq 2-4.
- `landing_wp_seq: 10` — same OFFBOARD-intervention pattern at `HOME_RETURN`
  for `FinalLanding`.
- `drop_wp_seq` left at its old value (10) but unused: `CruiseToDrop` /
  `DropOperation` / `ResumeAfterDrop` are now commented out of `KRAC_Mission_BT`'s
  `MissionSequence` in `krac_mission_bt_robust.xml` (definitions kept,
  unreferenced) until a real drop-zone waypoint/seq exists.

Updated in both `krac_control/config/krac_bt_params.yaml` and
`krac_control/bt/krac_mission_bt_robust.xml` (the XML's `seq="..."` attributes
are literal values actually consumed by the BT nodes at runtime — the yaml
`*_wp_seq` params are loaded into `MissionContext` but currently have no
reader, so keeping them in sync is documentation-only, not a functional
dependency).

## Other fixes (2026-07-05)

- **Legacy FSM vs BT separation**: `scripts/run_sitl.sh` (legacy FSM +
  mission_loader node) is unchanged. `scripts/run_sitl_bt.sh` now also starts
  `krac_bt_runner` itself (previously it only started PX4/MAVROS and left the
  BT runner to be launched manually in a second terminal).
- **vision_tracker → precision_lander topic gap**: `scripts/run_vision.sh`
  only launched `yolo_node`, so `precision_lander`'s `/vision/target_error`
  subscription (published only by `vision_tracker.py`) was never fed. Added
  `krac_vision/launch/vision_bt.launch.py` + `scripts/run_vision_bt.sh` to
  start `yolo_node` and `vision_tracker` together.
- **Image topic mismatch**: `vision_tracker.py` subscribed to a hardcoded
  `/camera/image_raw`, but `scripts/run_gz_image_republisher.sh` publishes
  `/image_raw` (same default already used by `yolo_detector.py`'s
  `image_topic` param). `vision_tracker.py` now takes the same `image_topic`
  parameter (default `/image_raw`).
- **survivor_tray Gazebo resolve failure**: `scripts/apply_px4_assets.sh`'s
  optional marker copy loop was `v_marker land_marker box victim` (missing
  `survivor_tray`) and only copied to `~/.gz/fuel/models`. Verified live in a
  running SITL session: `~/.gz/fuel/models` is **not** on `GZ_SIM_RESOURCE_PATH`
  (only `$PX4_DIR/Tools/simulation/gz/models` is, per `~/.bashrc`) —
  `box`/`victim`/`v_marker`/`land_marker` only ever resolved because copies of
  them already existed directly under `$PX4_DIR/Tools/simulation/gz/models`
  from an earlier manual copy, not from this script. Fixed by (1) adding
  `survivor_tray` to the loop and (2) copying every optional model to
  `$PX4_DIR/Tools/simulation/gz/models` as well as `~/.gz/fuel/models`.
  Confirmed fixed by respawning `survivor_tray` in a live world: the
  `Unable to find file with URI [model://survivor_tray/meshes/survivor_tray.stl]`
  error is gone after re-running `apply_px4_assets.sh`.
