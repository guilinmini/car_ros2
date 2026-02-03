from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	IncludeLaunchDescription,
	OpaqueFunction,
	SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
	pkg_share = get_package_share_directory("wheeltec_robot_urdf")
	default_urdf = os.path.join(pkg_share, "urdf", "V550_4wd_robot.urdf")

	urdf_arg = DeclareLaunchArgument(
		"urdf",
		default_value=default_urdf,
		description="Absolute path to robot URDF file",
	)
	gz_args_arg = DeclareLaunchArgument(
		"gz_args",
		default_value="-r empty.sdf",
		description="Gazebo args, e.g. '-r empty.sdf' or '-r -v 4 empty.sdf'",
	)

	gz_args = LaunchConfiguration("gz_args")

	def _create_state_publisher(context, *args, **kwargs):
		urdf_path = LaunchConfiguration("urdf").perform(context)
		with open(urdf_path, "r", encoding="utf-8") as f:
			robot_description = f.read()

		return [
			Node(
				package="robot_state_publisher",
				executable="robot_state_publisher",
				name="robot_state_publisher",
				output="screen",
				parameters=[
					{
						"use_sim_time": True,
						"robot_description": robot_description,
					}
				],
			)
		]

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
			"-topic",
			"/robot_description",
			"-x",
			"0",
			"-y",
			"0",
			"-z",
			"0.2",
		],
	)

	return LaunchDescription(
		[
			urdf_arg,
			gz_args_arg,
			set_gz_resource_path,
			gz_sim,
			OpaqueFunction(function=_create_state_publisher),
			spawn_entity,
		]
	)
