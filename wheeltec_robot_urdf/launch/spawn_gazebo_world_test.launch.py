from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	IncludeLaunchDescription,
	SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
	pkg_share = get_package_share_directory("wheeltec_robot_urdf")
	default_model = os.path.join(pkg_share, "sdf", "robot.sdf")
	default_world = os.path.join(pkg_share, "worlds", "car_world.sdf")

	model_arg = DeclareLaunchArgument(
		"model_file",
		default_value=default_model,
		description="Absolute path to robot SDF file",
	)
	gz_args_arg = DeclareLaunchArgument(
		"gz_args",
		default_value=f"-r {default_world}",
		description="Gazebo args, e.g. '-r world.sdf' or '-r -v 4 world.sdf'",
	)

	gz_args = LaunchConfiguration("gz_args")
	model_file = LaunchConfiguration("model_file")

	resource_paths = [os.path.dirname(pkg_share)]
	existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
	if existing_resource_path:
		resource_paths.append(existing_resource_path)

	set_gz_resource_path = SetEnvironmentVariable(
		"GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)
	)

	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory("ros_gz_sim"),
				"launch",
				"gz_sim.launch.py",
			)
		),
		launch_arguments={"gz_args": gz_args}.items(),
	)

	spawn_entity = Node(
		package="ros_gz_sim",
		executable="create",
		output="screen",
		arguments=[
			"-name",
			"wheeltec",
			"-file",
			model_file,
			"-x",
			"0",
			"-y",
			"0",
			"-z",
			"0.2",
		],
	)

	bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		output="screen",
		arguments=[
			"/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
			"/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
			"/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
		],
	)

	return LaunchDescription(
		[
			model_arg,
			gz_args_arg,
			set_gz_resource_path,
			gz_sim,
			spawn_entity,
			bridge,
		]
	)
