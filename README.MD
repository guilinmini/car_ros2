# Environment

## Installing the Default Gazebo/ROS Pairing
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

## Installing the `teleop_twist_keyboard`
```
sudo apt install ros-humble-teleop-twist-keyboard
```


# Usage
## start up Ignition Gazebo
```
ros2 launch wheeltec_robot_urdf spawn_gazebo.launch.py urdf:=path/to/your.urdf
```
## start up keyboard control node
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```