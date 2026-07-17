# KRAC_Mission_BT 구조 및 실행 흐름

대상 파일: `ros2_ws/src/krac_control/bt/krac_mission_bt_krac24_split.xml`
루트 트리: `KRAC_Mission_BT`

---

## 1. 최상위 제어 노드 계층

```
BehaviorTree "KRAC_Mission_BT"
└─ ReactiveFallback "RootReactiveFallback"        (:5)  ← 매 tick마다 첫 자식부터 재평가
   ├─ Sequence "EmergencyBranch"                  (:7)
   │   ├─ IsEmergencyDetected                     (:8)
   │   └─ EmergencyRecovery                       (:9)
   └─ Fallback "MissionOrHoldRecovery"             (:12)
      ├─ ReactiveSequence "SafetyGuardedMission"   (:13) ← 매 tick마다 첫 자식부터 재평가
      │   ├─ Sequence "SafetyGuard"                (:15)
      │   │   ├─ IsMAVROSConnected                 (:16)
      │   │   └─ IsBatterySafe                     (:17)
      │   └─ Sequence "MissionSequenceSplit"       (:20) ← 한 번 성공한 자식은 재실행 안 함("memory")
      │       └─ (2절 참고)
      └─ GlobalMissionRecovery                     (:145)
```

### 제어 노드 동작 차이 (중요)

- **`ReactiveFallback` / `ReactiveSequence`**: 매 tick마다 **항상 첫 번째 자식부터 다시 평가**한다 (memory 없음).
- **`Sequence`** (`MissionSequenceSplit`, `StartRescueLeg` 등): **memory가 있어서** 이미 SUCCESS한 자식은 다시 틱하지 않고, 현재 RUNNING 중인 자식부터 이어간다.

즉 `RescuePickupModule`처럼 몇 분씩 걸리는 액션이 RUNNING 중이어도, 매 tick마다:

- `IsEmergencyDetected` (:8) — 항상 최우선 재검사
- `IsMAVROSConnected` / `IsBatterySafe` (:16-17) — 계속 재검사

가 이루어진다. 하지만 이미 arm 했거나 미션을 이미 업로드한 단계는 다시 실행되지 않는다 — `MissionSequenceSplit` 자체가 memory 있는 `Sequence`라서 진행 상황을 기억하기 때문이다.

정리:

| 조건 | 결과 |
|---|---|
| 비상 감지 (`px4_failsafe,mavros_loss,gps_loss,battery_critical,manual_abort,geofence`) | 언제든 즉시 `EmergencyRecovery`로 전체 선점 (최우선순위) |
| MAVROS 연결 끊김 / 배터리 위험 | 언제든 즉시 `MissionSequenceSplit` 중단 → `GlobalMissionRecovery` |
| 그 외 | 미션 시퀀스가 처음부터 끝까지 한 번만 순서대로 진행 |

---

## 2. 미션 본문 (`MissionSequenceSplit`, :20-143)

### Phase 0 — 미션 업로드 / 시동 / AUTO.MISSION 진입 (`StartRescueLeg`, :22-47)

1. `ClearMissionPlan(optional=true)` (:24) — PX4에 남은 기존 미션 삭제 (없어도 무시)
2. `RetryUntilSuccessful(2)` → `UploadMissionPlan(krac24_rescue_leg.plan, mission_name=rescue_leg, expected_end=REP)` + `IsMissionUploaded` 검증 (:26-34) — 구조지점(REP)까지 가는 `.plan` 업로드, 최대 2회 재시도
3. `SetMissionPhase(0)` (:36) — 로그/상태용 phase 번호 발행
4. `RetryUntilSuccessful(3)` → `ArmVehicle(arm=true)` (:38-41) — 시동, 최대 3회
5. `RetryUntilSuccessful(3)` → `SetFlightMode(AUTO.MISSION)` (:43-46) — 자동 미션 비행 시작, 최대 3회

### Phase 1 — REP까지 자동 비행 (:49-51)

- `SetMissionPhase(1)`
- `WaitForWaypointReached(seq=6, match=after_start, timeout=600s)` — QGC 상 REP waypoint(#7, 0-index seq6) 도달까지 최대 10분 대기. 이 구간은 순수 AUTO.MISSION 자동비행이며 BT는 대기만 한다.

### REP 도착 → `TakeOverAtREP` (:53-70)

AUTO.MISSION → OFFBOARD로 **끊김 없이** 제어권을 넘기는 안전 절차.

1. `StartOffboardSetpointStream(hold_current_pose, 20Hz)` — 모드 전환 전 현재 위치 setpoint를 먼저 발행 시작 (PX4는 전환 요청 전부터 setpoint가 흐르고 있어야 OFFBOARD를 받아들임)
2. `Delay 1500ms` — 스트림이 몇 사이클 이상 실제로 발행될 시간 확보
3. `IsOffboardSetpointStreamAlive(min_rate_hz=10, stable_duration_sec=1.0)` — 스트림이 10Hz 이상으로 1초간 끊김 없이 나가는지 확인
4. `RetryUntilSuccessful(3)` → `SetFlightMode(OFFBOARD, timeout=8s)` — 실제 모드 전환
5. `ClearMissionPlan(optional=true)` — 남은 미션 plan 삭제 (미션 재개 오작동 방지)

### `SubTree ID="RescuePickupModule"` 호출 (:72) → 별도 트리(:150-162)

```
Sequence "RescuePlaceholderCycle"
├─ CommandVTOLTransition(MC, 20s)                              (:154)
├─ FlyToLocalPoint(-26.03, -31.41, 3.0m, ...)                   (:156)
├─ WaitForHoverStable(0.7m, 0.3m/s, 0.5m/s, 1.5s, 15s)          (:158)
└─ ExecuteExternalRescueModule(enable/ready/result 핸드셰이크)  (:160)
```

- MC(멀티콥터)로 전환 후, 로컬 ENU 상 고정 좌표(-26.03, -31.41, 3.0m — 구조물 상공 추정)까지 이동
- 그 지점에서 1.5초 이상 안정 호버 확인
- 이후 제어권을 통째로 외부 노드(`rescue_controller_placeholder.py`, 추후 control team 코드)에 넘김. 그리퍼 픽업/검증은 전부 그 안에서 처리되며 BT는 `result` 토픽만 기다린다 (최대 260초). 서브트리 결과(SUCCESS/FAILURE)가 그대로 `MissionSequenceSplit`의 다음 노드 결과가 된다.

> 상세 핸드셰이크 프로토콜(enable/ready/result 토픽, timeout, 제어팀 책임 범위)은 `docs/CONTROL_TEAM_EXTERNAL_RESCUE_INTERFACE.txt` 참고.

### Phase 9 — 귀환 미션 준비 (:73-91)

1. `SetMissionPhase(9)`
2. `RetryUntilSuccessful(2)` → `UploadReturnLeg`: `ClearMissionPlan` + `UploadMissionPlan(krac24_return_leg.plan, return_leg, expected_end=LANDING)` + `IsMissionUploaded` (:76-86)
3. `RetryUntilSuccessful(3)` → `SetFlightMode(AUTO.MISSION)` (:88-91) — 귀환 미션 시작
4. `WaitForWaypointReached(seq=1, after_start, timeout=180s)` (:93) — 귀환 경로 상 MC→FW 천이 지점(waypoint #2)까지 대기

### `AlignAndTransitionToFW` (:95-120) — 귀환 방향 정렬 후 고정익 전환

1. `TakeOverAtREP`와 동일한 패턴으로 OFFBOARD 안전 전환 (:97-108)
2. `WaitForHoverStable(altitude_tol=2.0m, ...)` (:111) — 픽업 후 상승한 상태의 호버 안정성 확인 (허용오차가 더 넉넉함, 페이로드 탑재 고려)
3. `AlignHeadingToWaypoint(seq=3, tol=0.20rad, timeout=30s)` (:113) — FW 전환 전 다음 목표(seq3) 방향으로 기수 정렬 (고정익 전환 시 옆미끄럼 방지)
4. `CommandVTOLTransition(FW, timeout=45s)` (:115) — 고정익 모드로 전환
5. `SetMissionCurrentWaypoint(seq=3, timeout=8s)` (:117) — AUTO.MISSION 재개 시 시작 waypoint 지정
6. `SetFlightMode(AUTO.MISSION, timeout=8s)` (:119) — 귀환 자동비행 재개

### Phase 10 — 최종 착륙 (:121-126)

- `WaitForWaypointReached(seq=7, after_start, timeout=600s)` — 버티포트(최종 착륙지점) waypoint 도달까지 대기
- `SetMissionPhase(10)`
- `DetectLanding(max_alt=0.4m, max_vz=0.3m/s, timeout=180s)` — PX4 `landed_state` 또는 저고도·저속도 조건으로 착륙 완료 판정

### 최종 disarm (`FinalDisarmOrAlreadyDisarmed`, :128-140)

`Fallback`이라 두 경로 중 하나만 통과하면 됨:

- 이미 disarm 상태 → `Inverter(IsVehicleArmed)` 바로 SUCCESS
- 아니면 → `ArmVehicle(arm=false, timeout=10s)` 실행 후 `Inverter(IsVehicleArmed)`로 실제 disarm 재확인

### Phase 11 — 임무 완료 표시 (:142)

`SetMissionPhase(11)`을 끝으로 `MissionSequenceSplit` 전체가 SUCCESS → 위로 전파되어 임무 종료.

---

## 3. 실패 시 흐름

`MissionSequenceSplit` 안 어느 단계든(각 `RetryUntilSuccessful`의 재시도 소진, 또는 `RescuePickupModule` 서브트리 FAILURE 등) 최종적으로 FAILURE가 나면:

```
Sequence 즉시 중단 → FAILURE
  → ReactiveSequence "SafetyGuardedMission"도 FAILURE
    → Fallback "MissionOrHoldRecovery"는 다음 자식 GlobalMissionRecovery 실행
      → GlobalMissionRecovery(reason="unhandled_mission_or_guard_failure")가 복구 처리
```

이 흐름과 별개로, `IsEmergencyDetected`는 매 tick 최우선으로 감시되므로 위 흐름 어느 지점에 있든 비상상황이 발생하면 즉시 `EmergencyRecovery`가 전체를 가로챈다.

---

## 4. Mission Phase 번호 요약

| Phase | 시점 |
|---|---|
| 0 | 구조 미션 업로드 + arm + AUTO.MISSION 진입 직후 |
| 1 | REP를 향해 자동비행 시작 |
| 9 | 픽업 완료, 귀환 미션 준비 시작 |
| 10 | 최종 착륙지점 도착, 착륙 판정 진입 |
| 11 | disarm 완료, 임무 종료 |
