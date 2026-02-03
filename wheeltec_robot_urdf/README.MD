# wheeltec_robot_urdf

## Overview

This package provides a URDF and launch helpers for running the Wheeltec robot in Ignition Gazebo (ros_gz). The instructions below show how to install required packages and run the robot with keyboard teleoperation.

## Prerequisites

- A ROS 2 distribution installed (set the `ROS_DISTRO` environment variable).
- Ignition Gazebo support for ROS (`ros_gz`) and `teleop_twist_keyboard`.

## Install (system packages)

Install the ros_gz bridge and the keyboard teleop package (replace `${ROS_DISTRO}` with your ROS 2 distro, e.g. `humble`):

```
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt-get install ros-${ROS_DISTRO}-teleop-twist-keyboard
```

Source your ROS environment (and the workspace overlays if you built this repo):

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
# if you built the workspace:
source install/setup.bash
```

## Usage

1. Launch the robot in Ignition Gazebo (provide the path to your URDF):

```bash
ros2 launch wheeltec_robot_urdf spawn_gazebo.launch.py urdf:=/path/to/your.urdf
```

2. Start keyboard teleoperation in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press the keys shown by `teleop_twist_keyboard` to publish velocity commands to `/cmd_vel`.

## Notes & Troubleshooting

- Replace `/path/to/your.urdf` with the actual filesystem path to your URDF file.
- If the launch fails, confirm the package `wheeltec_robot_urdf` is installed or the workspace is built (`colcon build`).
- Check that `ROS_DISTRO` is set correctly and that you sourced the appropriate `setup.bash` files.