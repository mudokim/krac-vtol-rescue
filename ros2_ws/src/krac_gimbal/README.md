# krac_gimbal

SIYI **A8 mini** 짐벌 제어 ROS 2 패키지.
`/gimbal/preset` 토픽으로 **1 / 2 / 3** 을 보내면 짐벌이 미리 정의된 프리셋 각도로 전환됩니다.

| 프리셋 | 의미 | 기본 각도 (yaw, pitch) |
|:---:|---|---|
| **1** | 중립 (전방 수평) | `(0°, 0°)` |
| **2** | 90도 (직하방) | `(0°, -90°)` |
| **3** | 180도 (요청) | `(0°, +25°)` ← *물리 한계로 대체, 아래 참고* |

---

## 1. 왜 이 방식인가 (A안: SIYI SDK 직접 제어)

기체 구성: **Pixhawk 6X (PX4) + Jetson Orin Nano** 를 이더넷으로 연결. A8 mini 도 제어보드가 이더넷을 출력합니다.

A8 mini 제어에는 두 갈래가 있는데:

| | **A. SIYI SDK 직접 (채택)** | B. MAVLink (Pixhawk 경유) |
|---|---|---|
| 제어 주체 | Jetson 이 UDP 로 짐벌에 직접 | PX4 gimbal 드라이버 + MAVROS |
| 지연 | 낮음 (Jetson ↔ 짐벌 직결) | 높음 (Jetson → FC → 짐벌) |
| 기능 | 절대각/속도/줌/촬영/자세피드백 전부 | 제한적 |
| 비전 연동 | YOLO 트래커와 Jetson 안에서 폐루프 | FC 왕복 필요 |

정밀착륙에서 짐벌이 하방 마커를 물고 있어야 하므로 **지연이 낮고 비전과 바로 폐루프가 되는 A안**을 채택했습니다.

---

## 2. 동작 원리

```
[YOLO 트래커]        [사용자/미션]
      │                   │ Int32(1/2/3)
      │                   ▼
      │            /gimbal/preset
      │                   │
      ▼                   ▼
┌───────────────────────────────────────┐
│            gimbal_node (Jetson)        │
│  ① 프리셋 번호 → 파라미터 각도 조회      │
│  ② 하드웨어 한계로 클램프               │
│  ③ SIYI SDK requestSetAngles(yaw,pit)  │
│  ④ hold_rate 로 저속 재전송(손실 대비)  │
│  ⑤ getAttitude() → /gimbal/attitude    │
└───────────────────────────────────────┘
        │  UDP 192.168.144.25:37260
        ▼
   [SIYI A8 mini]
```

- **절대각 제어**: `requestSetAngles(yaw_deg, pitch_deg)` 는 내부적으로 각도를 `×10` 한 정수로 바꿔 SIYI `SET_GIMBAL_ATTITUDE`(CMD `0x0e`) 패킷을 **한 번** 전송하는 비블로킹 명령입니다. 짐벌 펌웨어가 자체 안정화로 그 각도를 유지합니다.
- **유지(hold)**: UDP 는 손실될 수 있으므로, 활성 프리셋을 `hold_rate_hz`(기본 2Hz)로 다시 보내 확실히 도달·유지시킵니다. 같은 절대각을 반복 전송하는 것이라 지터는 없습니다.
- **자세 피드백**: SDK 백그라운드 스레드가 짐벌 자세를 받아오고, 노드가 `getAttitude()` 로 읽어 `/gimbal/attitude` 로 퍼블리시합니다.

> ⚠️ `setGimbalRotation()` 은 목표각까지 도는 **블로킹** 함수라 콜백에서 쓰면 실행기(executor)가 멈춥니다. 이 노드는 비블로킹인 `requestSetAngles()` 만 사용합니다.

---

## 3. "180도" 물리 한계 (중요)

A8 mini 의 각도 범위는 **pitch −90°~+25°**, **yaw −135°~+135°** 입니다 (SIYI SDK `cameras.py` 기준).

- `1`(중립 0°), `2`(직하방 −90°) 는 그대로 매핑됩니다.
- **`3`(180°) 는 어느 축으로도 물리적으로 불가능**합니다.
  - pitch 로 180° → 최대 상향이 **+25°** 뿐 (완전 뒤집기 불가)
  - yaw 로 180° → 최대 회전이 **±135°** 뿐

그래서 preset3 기본값은 **최대 상향 `(0, +25)`** 로 대체해 두었습니다.
의도가 **"뒤돌아보기(팬)"** 였다면 `config/gimbal_presets.yaml` 에서 `preset3: [135.0, 0.0]` 처럼 바꾸세요. 어떤 값이든 실행 시 하드웨어 한계로 **자동 클램프**되어 짐벌에 무리를 주지 않습니다.

---

## 4. 네트워크 구성

```
Pixhawk 6X ─ETH─┐
Jetson Orin ────┼── 이더넷 스위치
SIYI A8 mini ───┘
```

- A8 mini 는 `192.168.144.x` 대역(기본 IP `192.168.144.25`, 제어 UDP `37260`).
- Jetson 에 이 대역 IP를 부여하세요. 예: `192.168.144.20/24`.
  Pixhawk ETH 가 다른 대역이면 Jetson 인터페이스에 **IP 별칭 2개**를 주거나 SIYI Assistant 로 A8 IP 를 통일합니다.
- 영상(RTSP)은 `rtsp://192.168.144.25:8554/main.264` 로 들어옵니다. 이 패키지의
  **`camera_node`** 가 이 스트림을 받아 `/camera/image_raw` 로 퍼블리시하며, 이로써
  기존 `krac_vision` 트래커와 바로 연결됩니다. (각도 제어 UDP 와 영상 RTSP 는 같은
  A8 mini 지만 소프트웨어 경로가 완전히 분리되어 있습니다.)

---

## 5. ROS 2 인터페이스

**gimbal_node** (각도 제어)

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Sub | `/gimbal/preset` | `std_msgs/Int32` | 1/2/3 프리셋 전환 |
| Sub | `/gimbal/angle_cmd` | `geometry_msgs/Vector3` | 연속 절대각(x=yaw°, y=pitch°). 오토트랙/접근이 사용 |
| Pub | `/gimbal/attitude` | `geometry_msgs/Vector3Stamped` | 짐벌 자세(x=yaw, y=pitch, z=roll, 단위 °) |
| Srv | `/gimbal/center` | `std_srvs/Trigger` | 짐벌 센터 복귀(프리셋 유지 해제) |

**camera_node** (영상 브리지)

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Pub | `/camera/image_raw` | `sensor_msgs/Image` | A8 mini RTSP 영상(bgr8). `krac_vision` 이 구독 |

**approach_node** (짐벌-큐 접근, §7 참고)

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Sub | `/vision/target_error` | `krac_interfaces/TargetError` | YOLO 픽셀오차(1024px 기준) |
| Sub | `/gimbal/attitude` | `geometry_msgs/Vector3Stamped` | 짐벌 실제 자세 |
| Pub | `/gimbal/angle_cmd` | `geometry_msgs/Vector3` | 짐벌 비주얼 서보 명령 |
| Pub | `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | 기체 접근 속도 |
| Pub | `/approach/state` · `/approach/done` | `std_msgs/String` · `Bool` | 상태기계 상태 / 도달 플래그 |
| Srv | `/approach/start` | `std_srvs/SetBool` | 접근기동 arm/disarm (안전) |

파라미터는 [`config/gimbal_presets.yaml`](config/gimbal_presets.yaml)(각도),
[`config/camera.yaml`](config/camera.yaml)(영상),
[`config/approach.yaml`](config/approach.yaml)(접근) 참고.

---

## 6. 설치 · 빌드 · 실행

### 의존성 (Jetson)
```bash
pip install siyi-sdk          # 짐벌 제어(또는: git clone github.com/mzahana/siyi_sdk 후 PYTHONPATH 추가)
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv   # 카메라 브리지
```

### 빌드
```bash
cd ~/krac24        # 워크스페이스 루트
colcon build --packages-select krac_gimbal
source install/setup.bash
```

### 실행
```bash
# 짐벌 + 카메라 동시 (권장)
ros2 launch krac_gimbal bringup.launch.py

# 짐벌 각도 제어만
ros2 launch krac_gimbal gimbal.launch.py

# 카메라 영상 브리지만 (RTSP → /camera/image_raw)
ros2 launch krac_gimbal camera.launch.py

# 짐벌-큐 접근 코디네이터 (gimbal+camera+krac_vision 가 함께 떠 있어야 함)
ros2 launch krac_gimbal approach.launch.py

# 단독 실행
ros2 run krac_gimbal gimbal_node
ros2 run krac_gimbal camera_node
```

> Jetson 실기체에서는 저지연·저부하를 위해 [`config/camera.yaml`](config/camera.yaml)
> 에서 `backend: "gstreamer"` 로 바꿔 HW 디코드(nvv4l2decoder)를 쓰는 것을 권장합니다.

### 사용
```bash
# 2번(직하방)으로 전환
ros2 topic pub -1 /gimbal/preset std_msgs/msg/Int32 "{data: 2}"

# 1번(중립)
ros2 topic pub -1 /gimbal/preset std_msgs/msg/Int32 "{data: 1}"

# 센터 복귀
ros2 service call /gimbal/center std_srvs/srv/Trigger "{}"

# 자세 확인
ros2 topic echo /gimbal/attitude

# 카메라 영상 확인 (프레임률)
ros2 topic hz /camera/image_raw
# 영상 미리보기
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# 접근기동 시작/중단 (arm 전에는 기체에 아무 명령도 안 나감)
ros2 service call /approach/start std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /approach/state      # IDLE→TILT_INIT→SEARCH→APPROACH→OVERHEAD
ros2 service call /approach/start std_srvs/srv/SetBool "{data: false}"  # 중단
```

---

## 7. 짐벌-큐 접근 (approach_node)

**"60°에서 시작 → 전방 물체 인식 → 짐벌을 90° 직하로 정렬하며 물체 바로 위까지 접근"** 협조제어.

```
A8 mini RTSP → camera_node(/camera/image_raw)
            → krac_vision(YOLO, /vision/target_error)
            → approach_node ── /gimbal/angle_cmd ──▶ gimbal_node(짐벌이 타깃을 화면 중앙에 유지)
                           └─ /mavros/.../cmd_vel_unstamped ─▶ 기체(물체 위로 전진)
            → (OVERHEAD 도달) → precision_lander(기체 하강)
```

### 왜 이렇게 하면 되나 (원리)
- 짐벌 비주얼 서보가 타깃을 **항상 화면 중앙에 유지** → 짐벌 pitch = 타깃까지의 실제 **내림각(depression)** 이 된다.
- 기체가 전진해 수평거리 `d` 가 줄면 내림각은 기하학적으로 **60°→90°** 로 커진다.
- **전진속도 ∝ (90° − |pitch|)** 로 두면 머리 위(`d=0`, `pitch=−90°`)에서 속도가 0 → 정확히 그 위에서 정지.
- ⇒ "짐벌 직하 정렬"과 "물체 위 도달"이 동시에, 자연스럽게 완성된다.

### 상태기계
```
IDLE ─(/approach/start:true)─▶ TILT_INIT ─(짐벌 −60° 정착)─▶ SEARCH
  ▲                                                            │(연속 탐지)
  │(/approach/start:false, 언제든 중단)                        ▼
  └──────────────── OVERHEAD ◀─(직하 pitch≈−90 + 화면중앙 수렴)─ APPROACH
```

- **TILT_INIT / SEARCH**: 기체는 정지(호버), 짐벌만 전방 하향 `start_pitch`(기본 60°) 로 정렬 후 전방 탐색.
- **APPROACH**: 짐벌 서보(타깃 중앙 유지) + 기체 접근(전진속도 ∝ 90−|pitch|) 동시 구동. 타깃 소실 시 자동 정지.
- **OVERHEAD**: 수평 정지 + 짐벌 −90° 고정, `/approach/done=true` 발행. `handoff_precision_lander:true` 면 `/precision_lander/enable` 자동 호출.

게인·속도·임계값은 [`config/approach.yaml`](config/approach.yaml) 에서 조정합니다.
SITL 폐루프 시뮬레이션으로 `IDLE→…→OVERHEAD`(거리 2.89m→0m, pitch −60°→−90°) 수렴을 검증했습니다.

> ⚠️ 이 노드는 **기체 속도를 직접 명령**합니다. `/approach/start(true)` 로 arm 하기 전에는 아무 것도
> 내보내지 않으며, 실기 적용 전 반드시 SITL 에서 `lateral_sign`(횡방향 부호)과 게인을 검증하세요.

---

## 8. 트러블슈팅

| 증상 | 확인 |
|---|---|
| `siyi_sdk import 실패` | Jetson 에서 `pip install siyi-sdk` 또는 PYTHONPATH |
| 연결 실패 | `ping 192.168.144.25`, Jetson IP 가 `192.168.144.x` 대역인지, 방화벽/스위치 |
| 각도가 안 먹음 | A8 mini 펌웨어가 절대각(`SET_GIMBAL_ATTITUDE`) 지원 버전인지 확인 |
| 프리셋3 이 +25° 에서 멈춤 | 정상 (180° 물리 불가). §3 참고 |
| `/camera/image_raw` 안 나옴 | `ffplay rtsp://192.168.144.25:8554/main.264` 로 RTSP 확인, `cv_bridge`/`python3-opencv` 설치, `camera.yaml` 의 `rtsp_url` |
| 영상 지연 큼 | `camera.yaml` `backend: gstreamer`(Jetson HW 디코드) 또는 `rtsp_transport: udp` |
| `nvv4l2decoder` 없음 | PC/개발환경이면 `backend: ffmpeg` 사용(gstreamer 는 Jetson 전용) |

## 파일 구조
```
krac_gimbal/
├── package.xml
├── setup.py
├── README.md
├── config/
│   ├── gimbal_presets.yaml     # 프리셋 각도 + 네트워크 파라미터
│   ├── camera.yaml             # RTSP/디코드 파라미터
│   └── approach.yaml           # 짐벌-큐 접근 게인/임계값
├── launch/
│   ├── gimbal.launch.py        # 짐벌 각도 제어만
│   ├── camera.launch.py        # 카메라 영상 브리지만
│   ├── approach.launch.py      # 짐벌-큐 접근 코디네이터
│   └── bringup.launch.py       # 짐벌 + 카메라 동시
├── resource/krac_gimbal
└── krac_gimbal/
    ├── __init__.py
    ├── gimbal_node.py          # 프리셋(각도) 제어 노드
    ├── camera_node.py          # RTSP → /camera/image_raw 브리지 노드
    └── approach_node.py        # 60°→인식→90°직하 접근 상태기계
```
