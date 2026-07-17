# bt_real — 실기체 BT 운용 워크스페이스

`krac-vtol-rescue`(BehaviorTree 기반 VTOL 구조 미션, Gazebo/PX4-SITL 시뮬)를
**실제 드론용으로 정리·이관**한 워크스페이스. Gazebo 없이 실기체에서 동작하며,
Gazebo가 뜨지 않아도 **이미지 창(rqt_image_view)과 BT viewer 창**은 뜬다.

- 원본(시뮬) 그대로: `~/bt_ws/krac-vtol-rescue` (보존됨 — 비교/롤백용)
- 실제 카메라 검증본 `my_package`(SIYI RTSP) 를 통합

## 시뮬 → 실기체 변경 요약

| 구분 | 시뮬(krac-vtol-rescue) | 실기체(bt_real) |
| --- | --- | --- |
| 카메라 | Gazebo 카메라 → `gz_image_republisher`/`ros_gz_image` → `/image_raw` | **SIYI A8 RTSP** (`my_package` siyi_camera) → `/camera/image_raw` |
| 짐벌 | `gimbal_relay.py` + `ros_gz_bridge`(gz 조인트) | **SIYI SDK/UDP** (`krac_gimbal` gimbal_node) |
| FCU 연결 | `mavros_params.yaml` UDP 루프백(SITL) | `mavros_params_real.yaml` 실제 `fcu_url`(시리얼/UDP) |
| 기동 | `make px4_sitl` + auto_spawn + gz 브리지 (`run_sitl_bt.sh`) | **`run_real_bt.sh`** (SITL/Gazebo 요소 전부 제거) |
| 안전 파라미터 | `NAV_DLL_ACT=0`, `CBRK_AIRSPD_CHK` 강제(시뮬 크러치) | **제거**(실기체 PX4 preflight 사용) |
| BT/제어 토픽 | `/mavros/...` | **동일** (이름 안 바뀜 → 코드 그대로) |

> 카메라·비전 공유 토픽을 실기체 네이티브 규약 `/camera/image_raw` 로 통일했다
> (`my_package` 검증 기본값, `krac_gimbal/camera.yaml`, `landing_marker.py` 와 일치).

## 실행 (요약)

```bash
cd ~/bt_ws/bt_real/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash

cd ~/bt_ws/bt_real
./scripts/run_real_bt.sh      # MAVROS+카메라+짐벌+비전+rescue+그리퍼+BT + 이미지창 + BT viewer

# (별도 터미널) 착륙 후 그리퍼 수동 파지 — TTY(키보드) 필요라 자동으로 안 뜸
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 scripts/gripper_teleop.py     # Enter=파지확정, ESC=재시도
# 수동 파지 대기 자체를 건너뛰려면: MANUAL_GRASP=false ./scripts/run_real_bt.sh
```
**확인 프롬프트가 없다.** MAVROS 가 FCU 에 붙는 즉시 BT 가 OFFBOARD → ARM 으로 진행한다.
자세한 절차·토글·트러블슈팅은 **`docs/REAL_RUN_GUIDE.md`**.

## 실기체 그리퍼 (ESP32) — 2026-07-17 연동

제어팀 [`rionxst/payload_control`](https://github.com/rionxst/payload_control) 의 ESP32
시리얼 브리지를 쓴다. **비행 코드(BT / rescue_controller)는 수정하지 않았다** — 어댑터가
번역만 한다:

```
rescue_controller ──/model/standard_vtol_0/servo_4,5,6 (Float64, rad)──▶ gripper_adapter
  ──/gripper/set · /gripper/set_yaw · /gripper/arm (서비스)──▶ servo_serial_bridge
  ──시리얼 115200──▶ ESP32 (left=18, right=19, yaw=21)
```

`run_real_bt.sh` 가 자동으로 띄우고, **포트가 없으면 조용히 건너뛰며 미션은 그대로 진행**한다
(ESP32 없이 돌려도 종전과 동일 동작).

```bash
ls /dev/ttyUSB* /dev/ttyACM*                     # MAVROS 는 /dev/ttyTHS1 이라 충돌 없음
GRIPPER_PORT=/dev/ttyACM0 ./scripts/run_real_bt.sh   # 포트가 다르면
START_GRIPPER=false ./scripts/run_real_bt.sh         # 아예 끄려면
```

- **ARM 이 먼저다.** 펌웨어는 ARM 전 모든 명령을 `ERR,NOT_ARMED` 로 **조용히** 거부한다.
  어댑터가 시작 시 1회 ARM 하고, `NOT_ARMED` 응답을 보면(=ESP32 리셋) 재-ARM 한다.
- **핑거 임계 = -43도** (`gripper_open_deg`=-60 과 `gripper_grip_deg`=-26 의 중점).
  소스 주석("음수=열림, 양수=닫힘")과 달리 두 값이 **둘 다 음수**라 0 을 기준으로 잡으면
  파지 때마다 열려버린다. 두 파라미터를 바꾸면 어댑터의 `finger_open_threshold_deg` 도 같이.
- **파지력 조절은 안 된다**(펌웨어가 OPEN/CLOSE 이진). **접촉 피드백(`/gripper/contact`)도
  없다** → 파지 성공 확인은 `MANUAL_GRASP` 로 사람이.
- 회전이 반대로 돌면 `yaw_invert:=true`. 펌웨어 `setYaw` 는 약 3초 블로킹이라 어댑터가
  비동기 호출 + 3도 데드밴드를 둔다.

## ⚠️ 비행 전 필수 점검

실기체 배선/토픽은 맞춰뒀지만, **비행 로직/실측값/안전 게이트**는 현장 데이터와
제어팀 작업이 필요하다. 반드시 **`docs/REAL_HW_CHECKLIST.md`** 의 BLOCKER 를 먼저 처리:
1. `fcu_url` (실제 FC 연결)
2. 구조 상공 하드코딩 좌표 `(-26.03,-31.41,2.0)` (BT `:156`)
3. `.plan` 취리히(SITL) 좌표 → QGC 재작성
4. BT waypoint `seq` 번호 재산정
5. failsafe 강제해제 넣지 않기 (이미 제거됨)
6. ~~그리퍼 실드라이버 연동~~ → **2026-07-17 완료** (위 "실기체 그리퍼" 참조)
7. **카메라 캘리브레이션** — `precision_lander.cpp` 의 `fx_=582.5 / fy_=1036.7` 은
   **시뮬 카메라(PX4 스톡 mono_cam) 기준 실측 튜닝값이고 `const` 라 재컴파일 없이는
   못 바꾼다.** SIYI A8 로 바꾸면 정렬은 되는 것처럼 보이는데 엉뚱한 위치에 내려앉는다
   (실측: 오프셋 0.20 → 실제 0.36m 걸려 그리퍼가 0.16m 지나침). `fx_/fy_` 를 고치면
   `ALIGNED_RADIUS_M` / `APPROACH_RADIUS_M` / 데드밴드 / `lateral_scale` / 
   `grasp_offset_err_y_m`(0.111) 을 **전부 같이 재튜닝**해야 한다 (`REAL_HW_CHECKLIST.md` H10).

### 안전상 알아둘 것 (코드로 확인, 미수정)

- **BT 에 배터리 보호가 없다.** `IsBatterySafe` 는 `sim_bypass` 로 무조건 SUCCESS 이고
  `min_voltage`/`min_percentage` 를 읽지도 않는다 → PX4 `COM_LOW_BAT_ACT` 등이 유일한 방어선.
- **`IsOffboardSetpointStreamAlive` 는 `return SUCCESS` 한 줄**("TEMP BYPASS")이라 게이트가
  동작하지 않는다.
- **`GlobalMissionRecovery` 는 기본 `policy="hold"` 라 자동 착륙하지 않는다.** 가드가 걸리면
  제자리에 떠 있고 미션은 영구 중단된다(BT.CPP v3 Fallback 이 RUNNING child 를 latch 해서
  MAVROS 가 복구돼도 재개 안 됨) → **RC 킬 스위치 필수**. 착륙시키려면 XML 에
  `policy="land"` 를 준다(2026-07-17 구현).

## 구조

```
bt_real/
├─ README.md                     # (이 문서)
├─ docs/
│  ├─ REAL_RUN_GUIDE.md          # ★ 실행 가이드
│  ├─ REAL_HW_CHECKLIST.md       # ★ 실기체 전환 필수 점검
│  ├─ BT_MISSION_STRUCTURE.md, CONTROL_TEAM_*, krac24_integration.md, ...
├─ scripts/
│  ├─ run_real_bt.sh             # ★ 실기체 통합 기동(Gazebo 없음)
│  ├─ run_siyi_camera.sh         # SIYI 카메라 → /camera/image_raw
│  ├─ run_vision_bt.sh           # 비전 스택
│  ├─ gripper_teleop.py, collect_bt_debug_logs.sh, setup_ros2_ws.sh
└─ ros2_ws/src/
   ├─ my_package/                # SIYI RTSP 카메라(검증본) + siyi_camera.launch.py
   ├─ krac_gimbal/               # SIYI 짐벌(SDK/UDP) + camera_node
   ├─ krac_vision/               # vision_tracker / yolo / weights/best.pt
   ├─ krac_control/              # BT runner + bt/krac_mission_bt_krac24_split.xml
   ├─ krac_mission/              # real_vtol.launch.py + config/mavros_params_real.yaml
   ├─ servo_serial_bridge/       # ★제어팀 payload_control: ESP32 시리얼 브리지
   ├─ mission_interface/         # ★제어팀: SetServoAngle.srv (브리지가 의존)
   ├─ krac_interfaces/ krac_utils/ px4_msgs/ px4_ros_com/
```
> `krac_control/src/gripper_adapter.py` 가 우리 서보 토픽 ↔ 제어팀 서비스를 잇는다.
> 제어팀 저장소의 `my_package` 는 **우리 `my_package`(SIYI 카메라)와 이름이 충돌**하므로
> 가져오지 않았다.

## 빌드 주의 (젯슨)

`px4_msgs` 는 메시지 수천 개를 생성해서 젯슨(6코어/7.4GB)에서 빌드가 멈춘 것처럼 보인다.
**`krac_*` 중 `px4_msgs`/`px4_ros_com` 에 의존하는 패키지는 하나도 없으므로** 스킵한다:

```bash
MAKEFLAGS="-j3" colcon build --packages-skip px4_msgs px4_ros_com \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

정리 시 제거한 것: Gazebo 모델/월드(`models/`, `px4_assets/`), 미디어, 모든 시뮬/SITL
스크립트(`run_sitl*`, `auto_spawn*`, `gz_image_republisher*`, `apply_px4_assets`,
`gimbal_relay`, 이미지 브리지, `make_way3_*`), 시뮬 런치(`sitl_vtol`, `main_mission`),
시뮬 BT XML(krac24_split 외 전부) 및 각종 `.bak/.before_*` 백업, 시뮬 문서.
