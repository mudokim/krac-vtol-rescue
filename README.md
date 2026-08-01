# KRAC VTOL Rescue Mission

ROS 2, PX4, MAVROS, Gazebo, BehaviorTree.CPP, YOLO를 이용해 **VTOL 구조 임무를 처음부터 끝까지 하나의 Behavior Tree 기반 미션으로 연결하는 프로젝트**입니다.

이 저장소에서 제가 집중한 부분은 하드웨어 제작이 아니라, **전체 미션의 Behavior Tree 구조 설계와 소프트웨어 파이프라인 통합**입니다. PX4의 `AUTO.MISSION`과 `OFFBOARD` 제어를 연결하고, 비전/구조 제어 모듈을 독립적으로 호출한 뒤 결과를 받아 귀환과 최종 착륙까지 이어지도록 시스템을 구성했습니다.

## Current Status

- **로봇항공기대회 1차 예선 통과**
- Gazebo + PX4 SITL 환경에서 **Full Mission 시뮬레이션 완주**
- 시뮬레이션 구조를 실제 FCU, 카메라, 짐벌 환경으로 이식하는 작업 진행
- 현재 **2차 예선 발표 준비 중**

> `real_*` 브랜치는 실기체 환경에 맞춰 소프트웨어를 이식하고 안전하게 검증하기 위한 브랜치입니다. 현재 README에서는 실기체 전체 구조 미션 성공으로 표현하지 않습니다.

## My Role

- 초기 전체 **Behavior Tree 미션 구조 설계 및 수정**
- PX4 `AUTO.MISSION` ↔ `OFFBOARD` 제어권 전환 흐름 구성
- 구조 구간을 `RescuePickupModule`로 분리하고 Vision/Control 모듈과 연결
- 제어 모듈과 `enable → ready → result` 기반 hand-off 인터페이스 구성
- 구조 완료 후 return mission 업로드, MC → FW 천이, 최종 착륙까지 전체 흐름 연결
- 시뮬레이션 실행 구조를 실제 FCU / SIYI 카메라 / 짐벌 환경으로 이식
- 실기체에서 구조 구간만 반복 검증할 수 있도록 별도 테스트 BT 구성

## Mission Flow

```text
Mission Plan Upload
        ↓
ARM / AUTO.MISSION
        ↓
VTOL Rescue Site(REP) 이동
        ↓
OFFBOARD setpoint pre-stream
        ↓
AUTO.MISSION → OFFBOARD hand-off
        ↓
MC transition / hover stabilization
        ↓
RescuePickupModule
        ↓
External Rescue Module
(YOLO → target alignment → descent → rescue → ascent)
        ↓
SUCCESS / FAILURE result
        ↓
Return Mission Upload
        ↓
Heading Alignment
        ↓
MC → FW Transition
        ↓
AUTO.MISSION Return
        ↓
Final Landing / Disarm
```

## Behavior Tree Design

전체 미션은 단순한 순차 실행이 아니라, 안전 조건과 미션 진행 상태를 분리해 구성했습니다.

### Root / Safety

- `ReactiveFallback`을 사용해 emergency branch를 최우선으로 확인
- MAVROS connection, battery 등의 safety condition을 미션 실행 중 반복 확인
- 정상 흐름 실패 시 `GlobalMissionRecovery`로 전환

### Mission Sequence

- Rescue mission plan 업로드 및 검증
- Arm / `AUTO.MISSION`
- REP waypoint 도착 확인
- OFFBOARD setpoint를 먼저 발행한 뒤 제어권 전환
- `RescuePickupModule` 실행
- 구조 성공 후 return mission 업로드
- 귀환 방향 정렬 및 MC → FW transition
- 최종 착륙 및 disarm

## Control Team Hand-off

구조 알고리즘을 전체 BT 내부에 직접 넣지 않고 **외부 구조 모듈로 분리**했습니다.

BT는 구조 지점에서 기체가 안정적으로 hover한 뒤 외부 모듈에 제어권을 넘기고 결과만 받습니다.

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| BT → Rescue Module | `/krac/rescue_module/enable` | `std_msgs/Bool` | 구조 제어 시작/종료 요청 |
| Rescue Module → BT | `/krac/rescue_module/ready` | `std_msgs/Bool` | 자체 setpoint stream 준비 완료 |
| Rescue Module → BT | `/krac/rescue_module/result` | `std_msgs/String` | `SUCCESS` 또는 `FAILURE:reason` |

이 구조를 사용하면 구조 제어 알고리즘이 변경되어도 전체 미션, 귀환, 착륙 로직을 다시 작성하지 않고 **모듈 내부만 교체**할 수 있습니다.

## Branch Guide

프로젝트는 시뮬레이션 구현과 실기체 이식 단계에 따라 브랜치를 분리했습니다.

| Branch | Purpose | Main Contents |
| --- | --- | --- |
| [`main`](https://github.com/mudokim/krac-vtol-rescue/tree/main) | 프로젝트 진입점 | 전체 프로젝트 설명과 브랜치 안내 |
| [`refactor/split-mission-phases`](https://github.com/mudokim/krac-vtol-rescue/tree/refactor/split-mission-phases) | Full Mission BT 구조 정리 | Rescue leg / Rescue module / Return leg 분리, external rescue interface, full mission orchestration |
| [`yolo_test`](https://github.com/mudokim/krac-vtol-rescue/tree/yolo_test) | YOLO 구조 제어 통합 | YOLO detection, visual alignment, descent, ascent를 BT와 연결한 simulation test |
| [`real_full_scenario`](https://github.com/mudokim/krac-vtol-rescue/tree/real_full_scenario) | 전체 구조의 Sim → Real 이식 | Gazebo/SITL 요소 제거, 실제 MAVROS FCU, SIYI A8 camera, SIYI gimbal 실행 환경 |
| [`real_test`](https://github.com/mudokim/krac-vtol-rescue/tree/real_test) | 실기체 구조 구간 반복 검증 | 제자리 이륙 → 구조 모듈 → 재상승 → 착륙으로 범위를 축소한 안전 테스트 환경 |

### `refactor/split-mission-phases`

전체 미션을 한 노드가 모두 소유하지 않도록 책임을 분리한 핵심 브랜치입니다.

```text
Rescue Leg
→ OFFBOARD Hand-off
→ RescuePickupModule
→ External Rescue Module
→ Return Leg
→ VTOL Transition
→ Landing
```

BT는 전체 mission progression을 담당하고, 구조 제어팀은 `enable / ready / result` 인터페이스 안에서 구조 동작을 독립적으로 개발할 수 있도록 했습니다.

### `yolo_test`

Vision과 구조 제어를 BT에 실제로 연결해 end-to-end simulation을 검증한 브랜치입니다.

구조 제어 단계는 다음 상태로 분리되어 있습니다.

```text
IDLE
→ PREPARE_OFFBOARD
→ SEARCH_RESCUE
→ VISION_ALIGN_RESCUE
→ ASCEND_WITH_VICTIM
→ RESULT_HOLD
```

`/vision/target_error`의 pixel error를 이용해 타깃을 정렬하고, 조건이 만족되면 하강한 뒤 안전 고도로 재상승해 BT에 결과를 반환합니다.

### `real_full_scenario`

시뮬레이션에서 사용하던 Gazebo / PX4 SITL 의존 요소를 제거하고 실제 기체 환경에 맞춘 실행 구조를 구성했습니다.

주요 변경:

- PX4 SITL → 실제 FCU + MAVROS
- Gazebo camera → SIYI A8 RTSP camera
- Gazebo gimbal bridge → SIYI SDK/UDP gimbal control
- simulation launch → `run_real_bt.sh`
- simulation 전용 failsafe bypass 제거
- 실제 그리퍼 연결을 위한 adapter 구조 추가

BT와 `/mavros/...` 기반 인터페이스는 가능한 한 유지해 **미션 로직 자체를 다시 작성하지 않고 환경 의존 부분만 교체**하도록 구성했습니다.

### `real_test`

실기체 초기 테스트에서 전체 경로를 바로 비행하는 위험을 줄이기 위해 구조 구간만 분리한 브랜치입니다.

```text
현재 위치에서 이륙
→ 5 m 상승
→ RescuePickupModule
→ 다시 5 m 상승
→ AUTO.LAND
→ Disarm
```

큰 수평 이동을 제거해 Vision / 구조 제어 / PX4 연결을 좁은 범위에서 반복적으로 확인할 수 있도록 했습니다.

## Tech Stack

- Ubuntu 22.04
- ROS 2 Humble
- PX4
- MAVROS
- Gazebo Sim
- BehaviorTree.CPP
- YOLO
- QGroundControl
- C++ / Python
- SIYI A8 camera / gimbal integration for real-aircraft branches

## Sim-to-Real Notes

시뮬레이션에서 동작한 값을 그대로 실기체에 적용할 수 없기 때문에 실제 비행 전 다음 항목을 별도로 확인하도록 체크리스트를 구성했습니다.

- 실제 FCU connection / baud rate
- 실제 HOME / REP / return mission coordinates
- waypoint sequence 재검증
- PX4 failsafe / battery / OFFBOARD safety gate
- SIYI A8 camera calibration
- camera–vehicle–gripper extrinsic parameters
- 실제 기체 속도, hover, VTOL transition tuning

자세한 항목은 `real_full_scenario` 브랜치의 `docs/REAL_HW_CHECKLIST.md`를 참고합니다.

## What I Learned

이 프로젝트를 통해 단일 알고리즘을 구현하는 것보다 **여러 분야의 결과물이 하나의 시스템으로 연결되는 과정**을 경험했습니다.

하드웨어, 제어, 소프트웨어 팀이 서로 다른 책임을 가지고 개발하는 환경에서 어떤 인터페이스가 필요한지, 변경 사항을 어떻게 공유해야 하는지, 한 팀의 수정이 전체 시스템에 어떤 영향을 주는지 직접 경험했습니다.

또한 PX4, MAVROS, QGroundControl, BehaviorTree.CPP 등을 실제 프로젝트 흐름 안에서 처음부터 설정하고 사용하면서, 새로운 프레임워크를 빠르게 이해하고 시스템에 통합하는 경험을 쌓았습니다.