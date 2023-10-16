#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	imu_tranform_node = Node(
		package='imu_transform',
		executable='imu_transform',
		output='screen',
		shell=True,
	)

	return LaunchDescription([
		imu_tranform_node
	])
