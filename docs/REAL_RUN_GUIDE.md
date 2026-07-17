# bt_real 실기체 실행 가이드

`krac-vtol-rescue`(BT 기반 시뮬)를 실기체용으로 정리한 워크스페이스가 **bt_real** 이다.
Gazebo/PX4-SITL 없이 실제 드론에서 동작하며, **Gazebo가 뜨지 않아도 이미지 창과
BT viewer 창은 뜬다.**

시뮬 대비 바뀐 핵심:
- 카메라: Gazebo 카메라 브리지 → **SIYI A8 mini RTSP**(`my_package` siyi_camera) → `/camera/image_raw`
- 짐벌: Gazebo 조인트 릴레이 → **SIYI SDK(UDP)** (`krac_gimbal` gimbal_node)
- FCU 연결: SITL UDP 루프백 → **실제 FC**(`mavros_params_real.yaml` 의 `fcu_url`)
- 시뮬 전용 파라미터 조작(`NAV_DLL_ACT=0`, `CBRK_AIRSPD_CHK`) **제거**(안전)
- BT/제어(MAVROS 토픽)는 그대로 — 이름이 시뮬과 동일하기 때문

> ⚠️ **비행 전 반드시** `docs/REAL_HW_CHECKLIST.md` 의 BLOCKER 항목(fcu_url, 좌표,
> .plan, waypoint seq, failsafe, 그리퍼)을 처리할 것. 이 가이드는 **소프트웨어
> 기동 절차**이며, 실제 비행 안전 항목은 체크리스트에서 다룬다.

---

## 0. 사전 준비 (한 번만)

### 0-1. 네트워크 / 하드웨어
- **SIYI A8 mini**: 카메라/짐벌 모두 `192.168.144.25`.
  - Jetson 이더넷을 SIYI 서브넷(`192.168.144.x`, 예: `.20`)으로 설정.
  - 확인: `ping 192.168.144.25`
  - 카메라 RTSP: `rtsp://192.168.144.25:8554/main.264`
  - 짐벌 SDK: UDP `192.168.144.25:37260`
- **비행 컨트롤러(PX4)**: Jetson 과 UART(TELEM) 또는 USB 로 연결.
  - `mavros_params_real.yaml` 의 `fcu_url` 을 실제 배선에 맞게 설정(아래 0-3).

### 0-2. 시리얼 권한 (FC 를 시리얼로 붙일 때)
```bash
sudo usermod -aG dialout $USER   # 로그아웃 후 재로그인
ls -l /dev/ttyTHS1               # 또는 /dev/ttyACM0 등 실제 포트 확인
```

### 0-3. ★ fcu_url 설정 (필수) ★
`ros2_ws/src/krac_mission/config/mavros_params_real.yaml` 의 `fcu_url` 을 수정:
```yaml
# Jetson UART(TELEM) 직결:
fcu_url: "serial:///dev/ttyTHS1:921600"
# USB 직결:
# fcu_url: "serial:///dev/ttyACM0:57600"
# 네트워크/텔레메트리 UDP:
# fcu_url: "udp://:14540@<FC_IP>:14557"
```

### 0-4. 빌드
```bash
# (필수) BehaviorTree.CPP v3 — krac_control(BT 러너)가 find_package(behaviortree_cpp_v3) 함.
#        이 Jetson 엔 미설치 상태였음. v4(ros-humble-behaviortree-cpp)와 다른 패키지이니 v3 를 설치.
sudo apt install ros-humble-behaviortree-cpp-v3

cd ~/bt_ws/bt_real/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
> 빌드 확인(2026-07-17): `my_package`(SIYI 카메라)·`krac_interfaces`·`krac_gimbal` 은
> 정상 빌드됨. `krac_control` 은 위 `behaviortree_cpp_v3` 설치 후 빌드된다.
- `best.pt`(YOLO 가중치)는 `ros2_ws/src/krac_vision/weights/best.pt` 에 포함되어 있다
  (vtol-rescue BT 버전과 동일, md5 `3ee44046…`).
- (선택) drop-zone 검출용 `yolo_node` 를 쓸 때만 `vision_msgs` 필요:
  `sudo apt install ros-humble-vision-msgs` — **구조 미션(krac24_split)에는 불필요.**

---

## 1. 실행 (권장 터미널 구성)

모든 터미널에서 먼저:
```bash
cd ~/bt_ws/bt_real
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

### 터미널 1 — QGroundControl (선택)
```bash
~/QGroundControl.AppImage &
```
MAVROS 가 뜨기 전에 켜두면 미션/텔레메트리 확인이 편하다. (`gcs_url` 설정 시 중계됨)

### 터미널 2 — 메인 (BT + 카메라 + 짐벌 + 비전 + 뷰어)
```bash
./scripts/run_real_bt.sh
```
이 한 줄이 다음을 모두 띄운다 (**Gazebo 없이**):
| 구성요소 | 내용 |
| --- | --- |
| MAVROS | 실제 FC 연결 (`real_vtol.launch.py`) |
| SIYI 카메라 | RTSP → `/camera/image_raw` |
| SIYI 짐벌 | `gimbal_node` (SIYI SDK/UDP) |
| 비전 스택 | `vision_tracker`(+precision_lander) → `/vision/target_error`, `/vision/dbg_image` |
| rescue 컨트롤러 | 외부 rescue 모듈(핸드오프) |
| **🖥 이미지 창** | `rqt_image_view` on `/vision/dbg_image` |
| **🖥 BT viewer 창** | `krac_bt_runner` 내장 뷰어 (`enable_bt_viewer=true`) |
| BT | `krac_mission_bt_krac24_split.xml` 실행 |

> 두 GUI 창(이미지·BT viewer)은 **`DISPLAY`(X 디스플레이)** 가 있어야 뜬다.
> 헤드리스(SSH)면 자동으로 스킵되며, `ssh -X` 또는 VNC/모니터 연결 시 뜬다.

### 터미널 3 — 그리퍼 수동 파지 (선택, MANUAL_GRASP 시)
```bash
python3 scripts/gripper_teleop.py
```
> ⚠️ 현재 그리퍼 구동은 **Gazebo 서보 토픽 기반**이라 실기체 그리퍼는 움직이지
> 않는다. 실제 그리퍼 드라이버 연동 전까지는 파지 동작이 시뮬 수준이다
> (`docs/REAL_HW_CHECKLIST.md` BLOCKER #6).

---

## 2. 개별 실행 (디버깅용)

한 부분만 따로 확인하고 싶을 때:

```bash
# 카메라만 (RTSP → /camera/image_raw)
./scripts/run_siyi_camera.sh

# 비전 스택만 (카메라가 이미 떠 있어야 함)
ROS_IMAGE_TOPIC=/camera/image_raw ./scripts/run_vision_bt.sh

# 짐벌만
ros2 run krac_gimbal gimbal_node

# MAVROS 만
ros2 launch krac_mission real_vtol.launch.py

# 이미지 창만 (원본 카메라 보기)
ros2 run rqt_image_view rqt_image_view /camera/image_raw
# 추적 오버레이 보기
ros2 run rqt_image_view rqt_image_view /vision/dbg_image
```

---

## 3. 주요 환경변수 토글 (`run_real_bt.sh`)

| 변수 | 기본값 | 설명 |
| --- | --- | --- |
| `START_CAMERA` | `true` | SIYI 카메라 노드 |
| `START_GIMBAL` | `true` | SIYI 짐벌 노드 |
| `START_VISION_BT` | `true` | 비전 스택(tracker/precision_lander) |
| `START_RESCUE_PLACEHOLDER` | `true` | 외부 rescue 모듈(핸드오프) |
| `START_VISION_VIEW` | `true` | 🖥 이미지 창(rqt_image_view) |
| `ENABLE_BT_VIEWER` | `true` | 🖥 BT viewer 창 |
| `START_RQT_GRAPH` | `false` | 노드/토픽 그래프 |
| `ROS_IMAGE_TOPIC` | `/camera/image_raw` | 카메라·비전 공유 이미지 토픽 |
| `VISION_DEBUG_TOPIC` | `/vision/dbg_image` | 이미지 창에 띄울 토픽 |
| `MANUAL_GRASP` | `true` | 착륙 후 수동 파지 대기 |
| `BT_XML_PATH` | krac24_split | 실행할 BT XML |
| `RTSP_URL` | `rtsp://192.168.144.25:8554/main.264` | 카메라 스트림(서브: `/sub.264`) |

예)
```bash
ENABLE_BT_VIEWER=false ./scripts/run_real_bt.sh          # BT viewer 창 끄기
START_GIMBAL=false ./scripts/run_real_bt.sh              # 짐벌 없이
RTSP_URL=rtsp://192.168.144.25:8554/sub.264 ./scripts/run_real_bt.sh  # 저해상도 스트림
```

---

## 4. 토픽 흐름 (실기체)

```
[SIYI A8 RTSP] --siyi_camera--> /camera/image_raw (sensor_msgs/Image, bgr8, SensorDataQoS)
      |                                    |
      |                                    +--> vision_tracker --> /vision/target_error (krac_interfaces/TargetError)
      |                                                        \--> /vision/dbg_image  --> [🖥 rqt_image_view]
      |
[SIYI 짐벌 UDP] <--gimbal_node-- /gimbal/angle_cmd, /gimbal/preset ;  --> /gimbal/attitude

[PX4 FC] <==MAVROS(fcu_url)==> /mavros/state, /mavros/local_position/pose, /mavros/global_position/*,
                              /mavros/setpoint_velocity/cmd_vel_unstamped, /mavros/setpoint_raw/global,
                              /mavros/cmd/arming, /mavros/set_mode, /mavros/cmd/command, /mavros/mission/*
      ^
      +-- krac_bt_runner (BT) : 무장→AUTO.MISSION→REP→OFFBOARD 인계→VTOL 천이→
                                구조(외부 모듈 핸드오프)→복귀 leg→FW 천이→최종 착륙→disarm
      +-- rescue_controller_placeholder : /krac/rescue_module/{enable,ready,result}
```

---

## 5. 정상 동작 확인

```bash
# 카메라 프레임 수신(수 Hz~)
ros2 topic hz /camera/image_raw
# MAVROS ↔ FC 연결
ros2 topic echo /mavros/state --once | grep connected
# 추적 출력
ros2 topic echo /vision/target_error --once
# BT 전이 로그(터미널 2 출력) + BT viewer 창에서 현재 노드 상태 확인
```

로그 위치: `/tmp/krac_ros_logs/real_run_<timestamp>/`

---

## 6. 트러블슈팅

| 증상 | 원인 / 조치 |
| --- | --- |
| `MAVROS did not connect` | `fcu_url` 포트/보레이트/권한 확인(`0-3`, `dialout` 그룹). `ls /dev/tty*` 로 포트 확인 |
| 카메라 토픽 없음 | `ping 192.168.144.25`, RTSP 주소, 이더넷 서브넷 확인. `RTSP_TRANSPORT=udp` 시도 |
| 이미지 창/BT viewer 안 뜸 | `DISPLAY` 미설정(헤드리스). `echo $DISPLAY` 확인, `ssh -X` 또는 모니터/VNC |
| `vision_tracker` import 에러 | `krac_interfaces`(TargetError) 미빌드 → `colcon build` 재실행, `source install/setup.bash` |
| `yolo_node` 만 에러 | `vision_msgs` 미설치. 구조 미션엔 불필요(자동 스킵). 필요시 `apt install ros-humble-vision-msgs` |
| 짐벌 무반응 | `siyi_sdk` 설치·짐벌 IP 확인. `~/.local/lib/python3.10/site-packages/siyi_sdk` |

---

관련 문서
- `docs/REAL_HW_CHECKLIST.md` — 실기체 전환 필수 점검(좌표/plan/failsafe/캘리브레이션)
- `docs/BT_MISSION_STRUCTURE.md` — BT 미션 구조
- `docs/CONTROL_TEAM_EXTERNAL_RESCUE_INTERFACE.txt` — 외부 rescue 모듈 인터페이스
- `README.md` — bt_real 개요
