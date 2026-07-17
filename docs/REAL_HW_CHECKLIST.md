# 실기체 전환 필수 점검 체크리스트 (bt_real)

`krac_mission_bt_krac24_split.xml`(canonical 구조 미션) 흐름을 **처음부터 끝까지**
추적해 정리한, 실기체에서 반드시 확인/수정해야 하는 항목이다. 파일:라인은
`ros2_ws/src/...` 기준.

> 원칙: **소프트웨어 배선(토픽/런치)은 bt_real 에서 이미 실기체용으로 맞췄다.**
> 아래는 대부분 **비행 로직/실측값/안전 게이트** 항목으로, 실제 현장 좌표·기체
> 특성·제어팀 작업이 필요하다. 코드가 시뮬 값과 다르면 코드가 정답(문서 stale).

## 미션 흐름 요약 (krac24_split)
1. rescue leg 업로드 → 무장 → `AUTO.MISSION` 으로 REP 까지 자동비행(seq 6)
2. REP 상공에서 OFFBOARD 인계 → MC 천이 → `FlyToLocalPoint(-26.03,-31.41,2.0)` → hover
3. **외부 rescue 모듈** 실행(`/krac/rescue_module/*` 핸드오프, 최대 260s)
4. return leg 업로드 → `AUTO.MISSION`(seq 1) → 정렬 → FW 천이(seq 3) → 최종 착륙(seq 7) → disarm

---

## 🔴 BLOCKER — 처리 전 비행 금지

| # | 항목 | 위치 | 조치 |
| --- | --- | --- | --- |
| B1 | **fcu_url 이 SITL UDP 루프백** | `krac_mission/config/mavros_params_real.yaml` (bt_real 신규, 기본 `serial:///dev/ttyTHS1:921600`) | 실제 FC 시리얼/UDP·보레이트·`target_system_id`/`component_id` 확정 |
| B2 | **구조 상공 좌표가 하드코딩 로컬 ENU** `FlyToLocalPoint x=-26.03 y=-31.41 z=2.0` | `krac_control/bt/krac_mission_bt_krac24_split.xml:156` | EKF 로컬 원점은 부팅/GPS 시점마다 달라짐 → 고정값 제거하고 REP 글로벌좌표를 실시간 로컬로 변환(`bt_actions_precision.cpp:21-34` 헬퍼) 또는 글로벌 setpoint 사용 |
| B3 | **.plan 이 전부 취리히(SITL) 좌표** | `krac_control/src/krac24_rescue_leg.plan`, `krac24_return_leg.plan` (home `47.39…,8.54…`) | QGC 에서 HOME/REP/순항/복귀 waypoint·고도·양쪽 `DO_VTOL_TRANSITION` 재작성. 마지막 `NAV_VTOL_LAND` 항목 유지(미션 feasibility) |
| B4 | **BT waypoint seq 번호가 plan 항목 레이아웃에 강결합** | XML `:51`(seq6), `:93`(seq1), `:113/:117`(seq3), `:122`(seq7) | plan 수정 후 seq 값 전부 재산정(yaml 의 `*_wp_seq` 는 로드만 되고 실제로는 안 읽힘) |
| B5 | **안전 failsafe 를 무장 위해 강제 해제(시뮬 크러치)** `NAV_DLL_ACT=0`, `CBRK_AIRSPD_CHK` | (시뮬 `run_sitl_bt.sh`) — **bt_real `run_real_bt.sh` 에서 이미 제거함** | 유지: 실기체에서 이 조작을 넣지 말 것. PX4 자체 preflight(airspeed/heading/EKF)로 무장 판단 |
| B6 | **그리퍼 구동이 Gazebo 서보 토픽 전용** `/model/<m>/servo_4,5,6`, `/survivor_tray_rep/*` | `krac_control/src/rescue_controller_placeholder.py:359-367,630-637` | 실제 그리퍼 드라이버 + 접촉/파지 센서로 교체(핸드오프 계약은 유지) |

## 🟠 HIGH

| # | 항목 | 위치 | 조치 |
| --- | --- | --- | --- |
| H7 | **`IsOffboardSetpointStreamAlive` 가 항상 SUCCESS(영구 stub)** | `krac_control/src/bt_conditions.cpp:175-181` | OFFBOARD 인계(XML `:62`, `:104`) 전 setpoint 스트림 실제 검증 안 함 → 실 PX4 는 setpoint 부족 시 failsafe. 실제 `mission_context.cpp:553` `offboardStreamAlive()` 연결 |
| H8 | **비상 분기 무력(Emergency 항상 FAILURE)** | `bt_actions_recovery.cpp:25-30` | `px4_failsafe/mavros_loss/gps_loss/battery_critical/manual_abort/geofence` 실제 소스 구현. 현재 EmergencyBranch 데드코드 |
| H9 | **배터리 안전 우회(`sim_bypass=true`)** | XML `:17`, `config/krac_bt_params_krac24.yaml:12`, `bt_conditions.cpp:54-60` | 실제 `/mavros/battery` 임계치 배선 후 `sim_bypass=false`, `min_voltage/min_percentage` 설정 |
| H10 | **precision_lander 카메라 내부파라미터가 SITL 1024×1024 하드코딩** `fx=582.5, fy=1036.7, grasp_offset=0.111` | `krac_control/src/precision_lander.cpp:100-101,51,119-147` (파일 주석도 실카메라와 불일치 명시) | 실제 SIYI A8/RTSP 해상도·렌즈로 캘리브레이션, 기체-카메라-그리퍼 외부파라미터·하강게인 재튜닝 |
| H11 | **vision_tracker 해상도 가정 sim 고정(`img_size=1024`)** | `krac_vision/krac_vision/vision_tracker.py:73-75,124,191` | 실카메라 해상도/종횡비로 설정, `pixel_err` 스케일 재검증. ArUco 사전 `DICT_5X5_250` 고정(`:67-70`) 확인 |
| H12 | **구조 정밀착륙이 실비전으로 미검증 + 최종 파지가 수동(Enter)** | `HANDOFF_2026-07-06.md:113`; `rescue_controller_placeholder.py:1019-1113`; `MANUAL_GRASP` 기본 true | 제어팀이 placeholder 내부를 실제 검출/정렬/하강/파지/검증으로 교체(enable/ready/result 계약과 "복귀 시 armed+airborne+OFFBOARD+hover" 후조건 유지) |

## 🟡 MEDIUM

| # | 항목 | 위치 | 조치 |
| --- | --- | --- | --- |
| M13 | **stub-success C++ 기본값이 true** (params 없이 runner 뜨면 업로드/그리퍼 가짜 SUCCESS) | `mission_context.cpp:32-34` | 실기체 안전상 C++ 기본값을 false 로. params 파일 항상 전달(run_real_bt.sh 는 전달함) |
| M14 | **`mission_loader.py` 가 `mavros` 네임스페이스 하드코딩** | `krac_control/src/mission_loader.py:19-20` | MAVROS ns 다르면 업로드 실패 → 파라미터화 |
| M15 | **DetectLanding 이 상대고도(rel_alt) 임계 사용** | `bt_actions_mission.cpp:181-182,225` | 착륙지 지면고도가 이륙점과 다르면 오작동 → `landed_state==ON_GROUND` 우선 또는 임계 재튜닝 |
| M16 | **OFFBOARD hold 가 GPS 글로벌 기반, fix 없으면 무발행** | `mission_context.cpp:510-537` | 구조물 근처 GPS 저하 시 스트림 정지 → 로컬프레임 hold 폴백/ GPS 품질 게이팅 |
| M17 | **짐벌 IP/RTSP/캘리브레이션 실측 반영** | `krac_gimbal/config/camera.yaml`, `gimbal_presets.yaml`, `approach.yaml` | 실제 A8 IP·RTSP·짐벌 프리셋 각도 확인 |

## 🟢 LOW (sim 튜닝 상수 재검토)
- 타임아웃/속도: REP 대기 600s(XML `:51`), 복귀천이 180s(`:93`), 구조 260s(`:160`),
  `FlyToLocalPoint` max_xy 1.2/max_z 0.7(`:156`), FW 천이 45s(`:115`), P게인
  (`bt_actions_precision.cpp:354-361,416`) → 실기체 동역학·페이로드로 재튜닝.
- VTOL 천이(`bt_actions_basic.cpp:132-135`)는 param1 만 전송(immediate=0). MC→FW 전방
  천이 시 실기체 airspeed/throttle 여유 확인(스톨/quad-chute 주의).

## ✅ 실기체에서 그대로 안전한 부분
- 무장/모드/천이 노드는 표준 MAVROS 서비스(`/mavros/cmd/arming`, `/set_mode`,
  `/cmd/command`) + 실 상태 피드백 사용 → 메커니즘은 하드웨어 정합. (감싸는 게이트
  H7/H9 만 sim)
- 외부 rescue 핸드셰이크 프로토콜(`bt_actions_basic.cpp:299-520`)은 전송 무관·실기체 준비됨.
- `WaitForWaypointReached`/`SetMissionCurrentWaypoint`/`AlignHeadingToWaypoint` 는 실
  `/mavros/mission/*` + 실 GPS 방위 계산 사용 → 로직 정상(좌표 B3/seq B4 만 조정).
- BT 자체는 어떤 PX4 파라미터도 강제하지 않음(모든 param 조작은 시뮬 스크립트에만 있었고
  bt_real 에선 제거됨).
