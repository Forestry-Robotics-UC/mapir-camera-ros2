#!/usr/bin/env python3

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_rsp(context):
    urdf_file = LaunchConfiguration('urdf_file').perform(context).strip()
    with open(urdf_file, encoding='utf-8') as handle:
        robot_description = handle.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mapir_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    this_dir = os.path.dirname(__file__)
    default_urdf = os.path.normpath(
        os.path.join(this_dir, '..', 'config', 'mapir_survey3_visual.urdf')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Absolute path to MAPIR Survey3 URDF visual model.',
        ),
        OpaqueFunction(function=_build_rsp),
    ])
