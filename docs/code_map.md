# Code map

## PX4 / Gazebo assets

- `px4_assets/gz/models/amsr_vtol/model.sdf`: custom tiltrotor VTOL Gazebo model.
- `px4_assets/gz/worlds/my_world.sdf`: world that includes `model://amsr_vtol`.
- `px4_assets/airframes/1984_gz_amsr_vtol`: PX4 airframe script. Sets simulator model, VTOL control allocation, motor/servo mapping, and tuning parameters.
- `px4_assets/gz/models/v_marker`, `land_marker`, `box`, `victim`: competition objects spawned through Gazebo service.

## ROS 2 packages

### `krac_mission`

Launch/config package.

Important files:

- `launch/sitl_vtol.launch.py`: starts PX4 SITL, MAVROS, control node, mission loader, logger.
- `config/waypoints.yaml`: GPS waypoint set.
- `config/mavros_params.yaml`: MAVROS connection configuration.

### `krac_control`

Main control logic.

Important files:

- `src/vtol_fsm.cpp`: base FSM control node.
- `src/vtol_fsm_P.cpp`: P-turn / precision-control version used by launch.
- `src/transition_monitor.cpp`: phase/transition support node.
- `src/mission_loader.py`: loads QGC `.plan` mission into PX4 through MAVROS.

### `krac_vision`

Vision pipeline.

Important files:

- `krac_vision/yolo_detector.py`: YOLOv8 detector. Subscribes to `/image_raw`, publishes detections and `/camera/target_error`.
- `weights/best.pt`: trained YOLO model.
- `launch/yolo_detector.launch.py`: vision launch file.

### `krac_interfaces`

Custom message definitions.

- `FlightPhase.msg`
- `TargetInfo.msg`

### `krac_utils`

- `competition_logger.cpp`: mission logging utility.

## Runtime data flow

```text
PX4 SITL + Gazebo
    ↓ camera gz topic
scripts/run_image_bridge_amsr.sh
    ↓ /image_raw
krac_vision/yolo_detector.py
    ↓ /camera/target_error
krac_control/vtol_fsm_P.cpp
    ↓ MAVROS setpoints / services
PX4 flight stack
```
