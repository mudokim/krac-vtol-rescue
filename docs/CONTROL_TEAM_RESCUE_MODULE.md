# 제어·비전팀 RescuePickupModule 수정 가이드

## 1. 수정할 BT 위치

파일:

```text
ros2_ws/src/krac_control/bt/krac_mission_bt_krac24_split.xml
```

검색:

```xml
<BehaviorTree ID="RescuePickupModule">
```

이 `BehaviorTree` 내부가 제어·비전팀의 기본 수정 구역입니다.

메인 트리에는 다음 호출만 남습니다.

```xml
<SubTree ID="RescuePickupModule"/>
```

따라서 구조 알고리즘을 바꿔도 HOME→REP, return mission, FW 천이, 최종 착륙 순서는 그대로 유지됩니다.

---

## 2. 입력 상태

메인 BT가 모듈 호출 전에 보장합니다.

```text
MAVROS connected
vehicle armed
REP waypoint 도달
OFFBOARD mode
setpoint stream 정상
구조물 주변 멀티콥터 운용 가능
```

모듈 내부에서 다시 보장하는 항목:

```text
gripper open
VTOL MC state
REP 구조 위치 약 3 m 접근
hover stable
```

---

## 3. 출력 계약

### SUCCESS

다음 조건이 모두 충족돼야 합니다.

```text
gripper close 완료
simulation attach 또는 hardware contact 확인
payload를 붙인 채 lift 완료
OFFBOARD 유지
failsafe 없음
```

### FAILURE

반환하기 전에 다음 상태를 만들어야 합니다.

```text
precision lander off
continuous hold setpoint 유지
gripper open
payload detach
구조물과 충돌하지 않는 안전고도
hover stable
```

---

## 4. 단계별 수정 파일

### 구조 위치까지 가는 로직

BT 노드:

```xml
<FlyToLocalPoint .../>
<WaitForHoverStable .../>
```

C++:

```text
ros2_ws/src/krac_control/src/bt_actions_precision.cpp
```

수정 예:

```text
REP local x/y
접근 고도
XY/Z tolerance
최대 이동 속도
hover 안정 조건
```

### target 검출

BT 노드:

```xml
<ActivateYOLO .../>
<WaitForPrecisionTarget .../>
```

파일:

```text
ros2_ws/src/krac_vision/krac_vision/vision_tracker.py
ros2_ws/src/krac_interfaces/msg/TargetError.msg
ros2_ws/src/krac_control/src/bt_actions_vision.cpp
```

수정 예:

```text
class 이름
confidence
tracking hold
target freshness
pixel/metric error 정의
```

### 중심 정렬과 하강

BT 노드:

```xml
<PrecisionLandOnTarget .../>
```

제어기:

```text
ros2_ws/src/krac_control/src/precision_lander.cpp
```

완료 조건:

```text
ros2_ws/src/krac_control/src/bt_actions_precision.cpp
```

수정 예:

```text
XY PID
yaw 사용 여부
저고도 속도
target-loss 처리
touchdown/grip-ready 조건
고도 source
```

### 그리퍼 닫기

BT 노드:

```xml
<CloseGripper .../>
```

구현:

```text
ros2_ws/src/krac_control/src/bt_actions_basic.cpp
```

Gazebo 모델:

```text
px4_assets/model_patches/standard_vtol/model.sdf
```

수정 예:

```text
servo_5/servo_6 목표각
servo_4 회전 사용
close settle time
attach 시점
```

### 파지 확인과 상승

BT 노드:

```xml
<VerifyBasketPicked .../>
```

구현:

```text
ros2_ws/src/krac_control/src/bt_actions_vision.cpp
```

feedback:

```text
simulation: /survivor_tray_rep/gripper_state
hardware:   /gripper/contact
```

수정 예:

```text
lift 높이
contact stable time
attach timeout
lift 중 payload 유지 확인
```

### 실패 cleanup

BT 노드:

```text
<Sequence name="CleanupFailedRescueAttempt">
```

수정 예:

```text
detach
open gripper
복구고도
복구 속도
재시도 전 hover
```

---

## 5. 기존 노드만 사용할 때

XML만 수정합니다.

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split/ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select krac_control

source install/setup.bash
```

---

## 6. 새 BT 노드를 만들 때

다음 파일을 모두 수정해야 합니다.

```text
include/krac_control/bt/*.hpp
src/bt_actions_*.cpp
src/bt_register.cpp
CMakeLists.txt
bt/krac_mission_bt_krac24_split.xml
```

절차:

```text
1. StatefulActionNode 또는 SyncActionNode 클래스 작성
2. providedPorts 정의
3. bt_register.cpp 등록
4. CMakeLists.txt source 추가
5. RescuePickupModule에서 호출
6. TreeNodesModel에 port 문서 추가
7. build
```

---

## 7. 수정하면 안 되는 부분

제어·비전팀에서 직접 변경하지 않습니다.

```text
KRAC_Mission_BT의 Stage 1
WaitForWaypointReached seq6
TakeOverAtREP
UploadReturnLeg
return seq1/seq3/seq7/seq8
AlignAndTransitionToFW
FinalDisarmOrAlreadyDisarmed
EmergencyRecovery
GlobalMissionRecovery
```

필요하면 BT팀과 인터페이스를 먼저 합의합니다.

---

## 8. 현재 파지 모델 주의

현재는 물리적으로 십자 손잡이를 물어 고정하는 방식이 아닙니다.

```text
servo close
→ attach topic
→ detachable joint
```

따라서 다음 조건을 제어팀 코드에서 직접 추가하지 않으면 잘못된 위치에서도 attach할 수 있습니다.

```text
XY 중심 오차
고도 오차
yaw 오차
양쪽 finger contact
target freshness
```

권장 attach gate:

```text
center error <= 5 cm
vertical speed <= 0.15 m/s
horizontal speed <= 0.30 m/s
target fresh
gripper close 완료
```

---

## 9. 최소 검증 시나리오

```text
정중앙 시작
X +20 cm
X -20 cm
Y +20 cm
Y -20 cm
yaw ±30°
target 순간 상실 0.5초
target 상실 2초
attach 강제 실패
lift 중 detach
cleanup 후 다음 재시도
```

합격 기준:

```text
CloseGripper 중 OFFBOARD failsafe 0회
attach false positive 0회
실패 시 전복 0회
payload lift 성공률 90% 이상
```

---

## 10. 로그 확인

```bash
LOG_DIR=$(ls -td /tmp/krac_ros_logs/bt_run_* | head -1)

grep -RniE \
'RescuePickupModule|FlyToLocalPoint|WaitForPrecisionTarget|PrecisionLandOnTarget|CloseGripper|VerifyBasketPicked|attach|gripper_state|Failsafe|RTL|FAILURE|timeout' \
"$LOG_DIR" | tail -500
```
