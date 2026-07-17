# Environment setup

## 1. OS / ROS 2

Target: Ubuntu 22.04 + ROS 2 Humble.

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common git python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
```

Install ROS 2 Humble desktop using the official ROS 2 Ubuntu instructions, then source it:

```bash
source /opt/ros/humble/setup.bash
```

## 2. PX4-Autopilot

Original notes used:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
sudo reboot
```

For closer reproduction of the uploaded handoff, use PX4 v1.14.x + Gazebo Garden first:

```bash
cd ~/PX4-Autopilot
git checkout v1.14.3
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

Sanity test:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

## 3. Apply custom VTOL assets

From this repository:

```bash
cd ~/vtol-aam-rescue
./scripts/apply_px4_assets.sh
```

This copies:

- `px4_assets/gz/models/amsr_vtol` → `~/PX4-Autopilot/Tools/simulation/gz/models/amsr_vtol`
- `px4_assets/gz/worlds/my_world.sdf` → `~/PX4-Autopilot/Tools/simulation/gz/worlds/my_world.sdf`
- `px4_assets/airframes/1984_gz_amsr_vtol` → PX4 airframes folder
- marker models → `~/.gz/fuel/models`

Then test:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_amsr_vtol
```

## 4. ROS 2 dependencies

Likely needed packages:

```bash
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view
```

Install GeographicLib datasets for MAVROS if needed:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

For Gazebo Garden bridge, the original note used:

```bash
sudo apt install -y ros-humble-ros-gzgarden
```

For newer PX4/Harmonic environments, the package names may differ.

## 5. Python vision dependencies

```bash
python3 -m pip install --user ultralytics opencv-python torch numpy
```

## 6. Build workspace

```bash
cd ~/vtol-aam-rescue/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
