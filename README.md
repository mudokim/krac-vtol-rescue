# KRAC YOLO Test / Control Team Rescue

ROS 2 Humble, PX4 SITL, Gazebo, MAVROS, BehaviorTree, YOLO 기반의 VTOL 조난자 구조 테스트 브랜치입니다.

> 대상 브랜치: `yolo_test`

## 1. 처음 받는 사람 설치

### 필수 전제

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot이 기본적으로 `~/PX4-Autopilot`에 설치되어 있어야 함
- Gazebo Sim / MAVROS / ros_gz_bridge 설치
- QGroundControl AppImage 준비
- 저장소의 YOLO weight 파일 존재

저장소 받기:

```bash
mkdir -p ~/workspace/hzy
cd ~/workspace/hzy

git clone --branch yolo_test --single-branch \
  https://github.com/mudokim/krac-vtol-rescue.git \
  krac-control-team-complete

cd ~/workspace/hzy/krac-control-team-complete
find . -name "*.sh" -exec chmod +x {} \;
```

ROS 의존성 설치:

```bash
cd ~/workspace/hzy/krac-control-team-complete/ros2_ws
source /opt/ros/humble/setup.bash

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

PX4/Gazebo asset 적용:

```bash
cd ~/workspace/hzy/krac-control-team-complete
PX4_DIR=~/PX4-Autopilot ./scripts/apply_px4_assets.sh
```

asset 적용 후 PX4를 한 번 빌드합니다.

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_amsr_vtol
```

ROS 2 전체 워크스페이스 빌드:

```bash
cd ~/workspace/hzy/krac-control-team-complete/ros2_ws
source /opt/ros/humble/setup.bash

rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

> `setup.sh`는 현재 `krac_control` 중심 검증 스크립트입니다. 처음 받는 환경에서는 위처럼 전체 워크스페이스를 직접 빌드해야 `krac_vision`까지 함께 설치됩니다.

YOLO weight 확인:

```bash
ls -lh ~/workspace/hzy/krac-control-team-complete/ros2_ws/src/krac_vision/weights/best.pt
```

파일이 없으면 Vision stack이 시작되지 않습니다.

기본 검증:

```bash
cd ~/workspace/hzy/krac-control-team-complete
./verify.sh

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 pkg prefix krac_control
ros2 pkg prefix krac_vision
ros2 pkg prefix krac_interfaces
```

## 2. 실행

### 터미널 1: QGroundControl

PX4/MAVROS보다 먼저 실행하는 것을 권장합니다.

```bash
~/QGroundControl.AppImage &
```

QGC는 최초 vehicle 연결 시 mission list를 자동 요청하므로 SITL을 먼저 실행하고 QGC를 나중에 연결하면 지도에 waypoint가 바로 표시되지 않을 수 있습니다.

### 터미널 2: 전체 실행

```bash
cd ~/workspace/hzy/krac-control-team-complete

ENABLE_BT_VIEWER=true \
START_VISION_VIEW=true \
PRINT_BT_TRANSITIONS=true \
./run.sh
```

`run.sh`는 기존 PX4, MAVROS, BT, 구조 제어기 프로세스를 종료한 후 아래 실행 스크립트로 연결됩니다.

```text
run.sh
└── scripts/run_control_team_cycle.sh
    ├── rescue_offboard_bt_controller 실행
    ├── BT XML / parameter 파일 선택
    └── scripts/run_sitl_bt.sh 실행
```

주요 로그:

```bash
LOG_DIR=$(ls -td /tmp/krac_ros_logs/control_team_* 2>/dev/null | head -1)
echo "$LOG_DIR"

tail -f "$LOG_DIR/krac_bt_runner.launch.log"
tail -f "$LOG_DIR/rescue_offboard_bt_controller.log"
tail -f "$LOG_DIR/sitl_vtol.launch.log"
```

## 3. 실제 미션 흐름

현재 독립 테스트 BT는 다음 순서입니다.

```text
PX4 SITL 시작
→ outbound mission 업로드
→ AUTO.MISSION
→ ARM
→ VTOL_TAKEOFF waypoint seq 0 도달
→ OFFBOARD 인계
→ 구조 지점 (-26.03, -31.41, 3.0 m)으로 이동
→ hover 안정화
→ 외부 구조 모듈 enable
→ YOLO 탐색 / 중앙 정렬 / 하강 / 구조 판정 / 상승
→ 구조 모듈 SUCCESS
→ HOME (0, 0, 10 m) 복귀
→ AUTO.LAND
→ 착륙 및 disarm
```

## 4. 조난자 구조 로직을 수정할 때 볼 파일

파일이 많아 보여도 구조 로직 수정 시 핵심은 아래 3개입니다.

### 4.1 구조 지점 도착 시 어떤 BT가 실행되는지

```text
ros2_ws/src/krac_control/bt/krac_control_team_cycle_bt.xml
```

`RescuePickupModule`에서 구조 지점 이동과 외부 제어기 호출이 정의되어 있습니다.

```xml
<BehaviorTree ID="RescuePickupModule">
  <Sequence name="SharedControlTeamRescueModule">
    <CommandVTOLTransition target_state="MC" timeout_sec="20.0"/>

    <FlyToLocalPoint
      x_m="-26.03"
      y_m="-31.41"
      z_m="3.0"
      ... />

    <WaitForHoverStable ... />

    <ExecuteExternalRescueModule
      enable_topic="/krac/rescue_module/enable"
      ready_topic="/krac/rescue_module/ready"
      result_topic="/krac/rescue_module/result"
      ... />
  </Sequence>
</BehaviorTree>
```

즉:

1. BT가 구조 지점 좌표로 이동
2. hover 안정화 확인
3. `/krac/rescue_module/enable=true` 발행
4. 외부 구조 제어기가 실행됨
5. `/krac/rescue_module/result=SUCCESS`를 받으면 복귀 단계로 진행

구조 지점 좌표, 접근 고도, 허용 오차, 이동 속도를 바꾸려면 이 XML의 `FlyToLocalPoint`를 수정합니다.

### 4.2 실제 YOLO 기반 조난자 구조 제어 로직

```text
ros2_ws/src/krac_control/src/rescue_offboard_bt_controller.cpp
```

현재 실제 실행되는 구조 제어기는 이 C++ 노드입니다.

실행 연결:

```text
scripts/run_control_team_cycle.sh
└── ros2 run krac_control rescue_offboard_bt_controller
```

`CMakeLists.txt`에도 다음 executable로 등록되어 있습니다.

```cmake
add_executable(rescue_offboard_bt_controller
  src/rescue_offboard_bt_controller.cpp
)
```

구조 로직 상태는 다음과 같습니다.

```text
IDLE
→ PREPARE_OFFBOARD
→ SEARCH_RESCUE
→ VISION_ALIGN_RESCUE
→ ASCEND_WITH_VICTIM
→ RESULT_HOLD
```

주요 수정 위치:

#### `SEARCH_RESCUE`

YOLO가 대상을 찾기 전 탐색 및 탐색 고도 진입 로직입니다.

관련 함수/구간:

```cpp
executeSmoothSearchPattern(...)
case Phase::SEARCH_RESCUE:
```

탐색 방향, 탐색 패턴, 탐색 속도, 타겟 상실 시 복귀 동작을 바꾸려면 이 부분을 수정합니다.

#### `VISION_ALIGN_RESCUE`

YOLO에서 받은 pixel error를 속도 명령으로 바꾸고 중앙 정렬 및 하강을 수행하는 핵심 로직입니다.

현재 매핑:

```cpp
command.linear.x = clampVelocity(-vision_error_.y * vision_p_gain_);
command.linear.y = clampVelocity(-vision_error_.x * vision_p_gain_);
```

드론이 반대 방향으로 움직이거나 축이 뒤집혀 있으면 이 부호와 x/y 매핑을 먼저 확인합니다.

하강 조건:

```cpp
command.linear.z =
  (pixel_distance < descent_hold_error_px_) ? descend_speed_mps_ : 0.0;
```

중앙 정렬 허용 오차, 하강 시작 조건, 탐지 상실 처리, 저고도 dwell을 바꾸려면 이 상태를 수정합니다.

#### 구조 완료 판정

현재 구조 완료는 실제 그리퍼 물리 상태를 검사하는 방식이 아니라 다음 조건으로 확정됩니다.

```cpp
if (!grab_committed_ && detected &&
    alt < minimum_hover_altitude_m_ + 0.3)
{
  grab_committed_ = true;
  centered_started_ = now();
}
```

이후 `auto_proceed_after_hover_sec`만큼 대기하면 `ASCEND_WITH_VICTIM`으로 넘어갑니다.

따라서 실제 조난자 또는 구조 박스가 잡혔는지 검증하려면 이 파일에 다음을 추가해야 합니다.

- gripper close 명령 publish
- Gazebo attach 요청
- `/survivor_tray_rep/gripper_state` 또는 별도 gripper state subscribe
- 실제 attach/grip 성공일 때만 `grab_committed_=true`
- 실패 시 재정렬 또는 재시도

#### `ASCEND_WITH_VICTIM`

구조 완료 후 안전 고도까지 상승하고 안정화되면 `SUCCESS`를 발행합니다.

```cpp
case Phase::ASCEND_WITH_VICTIM:
```

상승 속도, 목표 고도, 수직 속도 안정 조건, SUCCESS 발행 조건을 바꾸려면 이 부분을 수정합니다.

### 4.3 수치만 튜닝할 때

```text
ros2_ws/src/krac_control/config/rescue_offboard_bt_params.yaml
```

알고리즘 구조를 바꾸지 않고 수치만 조정할 때는 이 파일을 수정합니다.

```yaml
vision_p_gain: 0.005
max_align_speed_mps: 0.5
descend_speed_mps: -0.3
search_speed_mps: 0.2
search_altitude_m: 3.0
minimum_hover_altitude_m: 2.0
center_tolerance_px: 60.0
descent_hold_error_px: 120.0
auto_proceed_after_hover_sec: 2.0
ascend_speed_mps: 0.5
ascend_target_altitude_m: 5.0
```

수정 기준:

| 현상 | 우선 수정할 값 |
|---|---|
| 중앙 정렬이 느림 | `vision_p_gain` 증가 |
| 좌우 흔들림이 큼 | `vision_p_gain`, `max_align_speed_mps` 감소 |
| 중심에 오기 전 하강 | `descent_hold_error_px` 감소 |
| 하강을 너무 못 함 | `descent_hold_error_px` 증가 |
| 너무 낮게 내려감 | `minimum_hover_altitude_m` 증가 |
| 파지 판정이 너무 빠름 | `auto_proceed_after_hover_sec` 증가 |
| 구조 후 상승이 느림 | `ascend_speed_mps` 증가 |

## 5. Vision 쪽을 수정할 때

구조 제어 C++는 아래 토픽을 입력으로 사용합니다.

```text
/vision/target_error
krac_interfaces/msg/TargetError
```

주요 필드:

```text
is_detected
pixel_err_x
pixel_err_y
yaw_err_rad
```

YOLO 모델, confidence, OBB 추적, pixel error 계산을 수정하려면 `ros2_ws/src/krac_vision` 패키지를 확인합니다.

현재 구조 제어기의 타겟 라벨은:

```text
basket
```

이며 `/camera/set_target`로 전달됩니다.

## 6. 잘못 수정하기 쉬운 파일

```text
ros2_ws/src/krac_control/src/rescue_controller_team.py
```

이 파일은 템플릿/이전 공유 제어기 성격의 파일이며, 현재 `run_control_team_cycle.sh`에서 실제 실행하는 구조 제어기는 아닙니다.

현재 실행 정본은 반드시 다음 파일을 기준으로 봅니다.

```text
ros2_ws/src/krac_control/src/rescue_offboard_bt_controller.cpp
```

## 7. 코드 수정 후 빌드

C++ 구조 제어기 또는 BT/XML/config 수정 후:

```bash
cd ~/workspace/hzy/krac-control-team-complete/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to krac_control
source install/setup.bash
```

Vision Python 코드 수정 후에도 전체 overlay를 다시 source합니다.

```bash
cd ~/workspace/hzy/krac-control-team-complete/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 8. 실행 상태 확인

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/hzy/krac-control-team-complete/ros2_ws/install/setup.bash
```

```bash
ros2 node list
ros2 topic echo /mavros/state --once
ros2 topic echo /vision/target_error
ros2 topic echo /krac/rescue_module/ready
ros2 topic echo /krac/rescue_module/result
```

구조 모듈만 강제로 시작:

```bash
ros2 topic pub --once \
  /krac/rescue_module/enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

수동으로 다음 단계 진행:

```bash
ros2 service call /cmd/mission_proceed std_srvs/srv/Trigger "{}"
```

## 9. Git 반영

현재 브랜치는 `yolo_test`입니다.

```bash
cd ~/workspace/hzy/krac-control-team-complete

git status
git add -A
git commit -m "Update YOLO rescue logic"
git push mudokim yolo_test
```

다른 사용자가 최신 내용을 받을 때:

```bash
cd ~/workspace/hzy/krac-control-team-complete
git pull --rebase mudokim yolo_test
```
