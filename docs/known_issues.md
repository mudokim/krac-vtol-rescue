# Known issues / cleanup checklist

## 1. Hard-coded `/home/kch` paths

Original code includes hard-coded paths such as:

- `/home/kch/ros2_ws/src/krac_control/src/way3.plan`
- `/home/kch/ros2_ws/src/krac_vision/weights/best.pt`

These should be changed to package-relative paths using `ament_index_python` or launch parameters.

## 2. Image topic mismatch

Original note sometimes bridges Gazebo camera to `/camera`, but `yolo_detector.py` subscribes to `/image_raw`.

For this cleaned repo, `scripts/run_image_bridge_amsr.sh` remaps the Gazebo camera directly to `/image_raw`.

## 3. Gazebo version ambiguity

The handoff notes mention both:

- Gazebo Harmonic through `PX4-Autopilot/Tools/setup/ubuntu.sh`
- Gazebo Garden 7.9.0 / `ros-humble-ros-gzgarden`

For first reproduction, use PX4 v1.14.x + Garden. For long-term maintenance, migrate to PX4 main + Harmonic.

## 4. Custom airframe CMake registration

`1984_gz_amsr_vtol` must be listed in:

```text
~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

`apply_px4_assets.sh` tries to patch this automatically, but manually check if `make px4_sitl gz_amsr_vtol` fails.

## 5. MAVROS connection may need launch delay

`mission_loader.py` uploads the mission when the launch starts. If MAVROS is not connected yet, mission upload can fail. A launch delay or service retry loop may be needed.

## 6. Public repo warning

The raw handoff contained a Roboflow API key in a docx note. Do not commit that note or key to a public repository.
