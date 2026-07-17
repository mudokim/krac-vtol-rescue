# bt_rescue_test — REP 구조 "제자리" 테스트 워크스페이스

`bt_real`(전체 미션)과 **완전히 분리된 독립 워크스페이스**. REP 구조 로직만 제자리에서
반복 검증한다. 여기서 검증되면 `RescuePickupModule` 을 전체 시나리오(bt_real)에 적용하면 된다.

- BT: `ros2_ws/src/krac_control/bt/bt_rescue_test.xml` (이 폴더엔 이 BT 하나만)
- 실행: `scripts/run_rescue_test.sh`

## 시나리오 (전부 제자리)

기체를 **REP(트레이/조난자) 바로 옆 ~1m**에 놓고 시작. 좌표 지정 없이 YOLO/비전이 추적.

```
1) 제자리 5m 이륙   : OFFBOARD 진입 → arm → FlyToAltitude(5m, 현재 위치 유지)
2) 구조             : MC 보장 → hover 안정 → ExecuteExternalRescueModule
                      (비전으로 타깃 정렬 → 정밀 하강 → (파지) → 재상승)
3) 다시 5m 상승     : FlyToAltitude(5m, 제자리) → hover 안정
4) 제자리 착륙      : AUTO.LAND → 착륙 판정 → disarm
```
* BT 가 큰 수평 이동을 명령하지 않음(수직 + 비전 정렬만) → 안전.
* `FlyToLocalPoint`(좌표 비행) 제거함.

## 사전 준비

```bash
# 빌드 의존성 (이 Jetson 에 없던 것)
sudo apt install ros-humble-behaviortree-cpp-v3   # (설치 완료)
sudo apt install libsdl2-ttf-dev                  # ★ BT viewer 렌더링용, 아직 필요

cd ~/bt_ws/bt_rescue_test/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```
- `fcu_url`: `ros2_ws/src/krac_mission/config/mavros_params_real.yaml` 확인(실제 FC).
- 카메라: SIYI A8 `192.168.144.25` (`ping` 확인). 비전이 `/camera/image_raw` 구독.
- `best.pt`: `krac_vision/weights/best.pt` 포함됨.

## 실행

```bash
cd ~/bt_ws/bt_rescue_test
./scripts/run_rescue_test.sh
```
뜨는 것(Gazebo 없음): MAVROS + SIYI 카메라 + 짐벌 + 비전 + rescue 컨트롤러 +
**그리퍼(ESP32 연결 시)** + **BT viewer 창** + **rqt_image_view 이미지 창**.

**확인 프롬프트가 없다.** MAVROS 가 FCU 에 붙는 즉시 BT 첫 tick 부터
OFFBOARD → ARM → 5m 상승이 이어진다. 명령 치는 순간이 곧 이륙이다.

주요 토글:

| 변수 | 기본 | 설명 |
|---|---|---|
| `START_GRIPPER` | `true` | 실기체 그리퍼. **포트 없으면 자동 스킵** |
| `GRIPPER_PORT` | `/dev/ttyUSB0` | ESP32 USB-시리얼 |
| `MANUAL_GRASP` | `true` | 파지를 사람이 확인 |
| `START_CAMERA` | `true` | SIYI RTSP |
| `START_YOLO_DETECTOR` | `true` | 이 BT 는 안 씀 → `false` 로 꺼도 무방 |

## ★ 그리퍼 수동 파지 조작은 어떻게 켜나

**`run_rescue_test.sh` 는 그리퍼 teleop 을 자동으로 띄우지 않는다.** 이유: `gripper_teleop.py`
는 키보드를 raw(TTY)로 읽어야 해서 백그라운드로 못 띄운다 → **반드시 별도 터미널**에서
직접 실행해야 한다. (그래서 run 스크립트는 `MANUAL_GRASP=true` 일 때 안내 문구만 출력한다.)

```bash
# 별도 터미널에서
cd ~/bt_ws/bt_rescue_test
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 scripts/gripper_teleop.py
```
- `Enter` → `/krac/manual_grasp/confirm` (파지 확정, 상승으로 진행)
- `ESC`   → `/krac/manual_grasp/retry` (실패, 이 구간 재시작)
- 집게 키(w/s/o/g 등) → `/krac/gripper/cmd`

**(2026-07-17)** 실기체 그리퍼가 붙었다. ESP32 가 꽂혀 있으면 `Enter`/집게 키로 **실제
집게가 움직인다.** 아래 "실기체 그리퍼" 참조. ESP32 가 없으면 예전처럼 서보는 안 움직이고
`Enter`/`ESC` 로 흐름만 진행된다.

사람 입력 대기로 미션을 멈추고 싶지 않으면 수동 파지를 끈다:
```bash
MANUAL_GRASP=false ./scripts/run_rescue_test.sh
```
→ 타깃 위 착지 후 바로 재상승(파지 동작 생략).

## ★ 실기체 그리퍼 (ESP32)

제어팀 [`rionxst/payload_control`](https://github.com/rionxst/payload_control) 의 ESP32
시리얼 브리지를 쓴다. **비행 코드는 수정하지 않았다** — 어댑터가 번역만 한다:

```
rescue_controller ──/model/standard_vtol_0/servo_4,5,6 (Float64, rad)──▶ gripper_adapter
  ──/gripper/set · /gripper/set_yaw · /gripper/arm (서비스)──▶ servo_serial_bridge
  ──시리얼 115200──▶ ESP32 (left=18, right=19, yaw=21)
```

`run_rescue_test.sh` 가 자동으로 띄운다. **`GRIPPER_PORT` 가 없으면 조용히 건너뛰고
미션은 그대로 진행**하므로, ESP32 없이 돌려도 지금까지와 동일하게 동작한다.

```bash
ls /dev/ttyUSB* /dev/ttyACM*                        # 포트 확인 (MAVROS 는 /dev/ttyTHS1 라 충돌 없음)
GRIPPER_PORT=/dev/ttyACM0 ./scripts/run_rescue_test.sh   # 포트가 다르면
START_GRIPPER=false ./scripts/run_rescue_test.sh         # 아예 끄려면
```

알아둘 것:

- **ARM 이 먼저다.** 펌웨어는 ARM 전 모든 명령을 `ERR,NOT_ARMED` 로 **조용히** 거부한다.
  어댑터가 시작 시 1회 ARM 하고, `NOT_ARMED` 응답을 보면(=ESP32 리셋) 재-ARM 한다.
- **핑거 임계 = -43도.** `rescue_controller` 의 `gripper_open_deg`(-60) 과
  `gripper_grip_deg`(-26) 의 중점이다. 소스의 "음수=열림, 양수=닫힘" 주석과 달리 두 값이
  **둘 다 음수**라 0 을 기준으로 잡으면 파지 때마다 열려버린다. 두 파라미터를 바꾸면
  어댑터의 `finger_open_threshold_deg` 도 같이 바꿔야 한다.
- **파지력 조절은 안 된다.** 펌웨어가 OPEN/CLOSE 이진이라 각도로 무는 힘을 못 준다
  (우리 각도-틈 매핑은 Gazebo 모델 기구학 전용이라 실기체엔 원래 해당 없음).
- **회전 서보가 반대로 돌면** `yaw_invert:=true`. 펌웨어 `setYaw` 는 각도와 무관하게
  **약 3초 블로킹**이라(10단계 × 0.3초) 어댑터는 비동기로 부르고 3도 데드밴드를 둔다.
- **접촉 피드백(`/gripper/contact`)이 없다.** 파지 성공을 기계적으로 확인할 수 없어
  `MANUAL_GRASP` 로 사람이 눈으로 봐야 한다.

## 주의 (docs/REAL_HW_CHECKLIST.md 참조)
- 비전 정렬 정확도는 `precision_lander.cpp` 의 `fx_/fy_`(현재 시뮬 카메라 값)에 좌우 →
  실제 SIYI A8 로 캘리브레이션해 맞춰야 정렬이 정확 (H10).
- 이륙/재상승은 GPS(상대고도) 기반 → GPS fix 필요.
- 실기체 무장 게이트는 PX4 preflight 로 판단(시뮬용 파라미터 우회 없음).
