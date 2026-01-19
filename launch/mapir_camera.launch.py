#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2025 Duda Andrada
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: GNU General Public License v3.0 (GPL-3.0-only)
# Repository: mapir_survey3
#
# Description:
#   Launch MAPIR Survey3 camera node with proper frame_id conventions.
#   Publishes under /<namespace> (e.g., /mapir/image_raw and /mapir/camera_info).
#
# Notes (REP-105 / optical frame conventions):
#   Optical frame uses:
#     +X to the right in the image
#     +Y down in the image
#     +Z forward (out of the camera)
#   If you do not yet have a URDF, an optional static transform from mapir3_link
#   to mapir3_optical_frame is a practical placeholder.

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mapir_camera_ros2.core.spectral_indices import supported_spectral_indices


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('mapir_camera_ros2')

    default_reflectance_params_file = os.path.join(pkg_share, 'config', 'reflectance_ocn.yaml')
    default_reflectance_params = (
        default_reflectance_params_file
        if os.path.exists(default_reflectance_params_file)
        else ''
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mapir',
        description='Namespace for camera node and topics (e.g., mapir).',
    )

    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value='',
        description='Path to a ROS 2 params YAML file for camera_node.',
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description='Override camera_info_url (file://... YAML).',
    )

    camera_impl_arg = DeclareLaunchArgument(
        'camera_impl',
        default_value='py',
        description='Camera implementation: "py" (camera_node) or "cpp" (camera_node_cpp).',
    )

    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='true',
        description='Publish placeholder static TF mapir3_link -> mapir3_optical_frame.',
    )

    static_tf_parent_arg = DeclareLaunchArgument(
        'static_tf_parent_frame',
        default_value='mapir_link',
        description='Static TF parent frame.',
    )

    static_tf_child_arg = DeclareLaunchArgument(
        'static_tf_child_frame',
        default_value='mapir_optical_frame',
        description='Static TF child frame.',
    )

    static_tf_x_arg = DeclareLaunchArgument(
        'static_tf_x',
        default_value='0',
        description='Static TF x (m).',
    )
    static_tf_y_arg = DeclareLaunchArgument(
        'static_tf_y',
        default_value='0',
        description='Static TF y (m).',
    )
    static_tf_z_arg = DeclareLaunchArgument(
        'static_tf_z',
        default_value='0',
        description='Static TF z (m).',
    )

    static_tf_roll_arg = DeclareLaunchArgument(
        'static_tf_roll',
        default_value='1.5707963',
        description='Static TF roll (rad).',
    )

    static_tf_pitch_arg = DeclareLaunchArgument(
        'static_tf_pitch',
        default_value='0',
        description='Static TF pitch (rad).',
    )

    static_tf_yaw_arg = DeclareLaunchArgument(
        'static_tf_yaw',
        default_value='3.1415926',
        description='Static TF yaw (rad).',
    )

    enable_indices_arg = DeclareLaunchArgument(
        'enable_indices',
        default_value='false',
        description='Enable spectral indices processing node.',
    )

    indices_params_file_arg = DeclareLaunchArgument(
        'indices_params_file',
        default_value='',
        description='YAML params file for indices_node (leave empty to use node defaults).',
    )

    indices_per_node_arg = DeclareLaunchArgument(
        'indices_per_node',
        default_value='false',
        description='Launch one indices_node per index (avoids multi-index bottleneck).',
    )

    indices_all_arg = DeclareLaunchArgument(
        'indices_all',
        default_value='false',
        description='Use all supported indices instead of the list from indices_params_file.',
    )

    enable_reflectance_arg = DeclareLaunchArgument(
        'enable_reflectance',
        default_value='false',
        description='Enable reflectance calibration node.',
    )

    reflectance_debug_arg = DeclareLaunchArgument(
        'reflectance_debug',
        default_value='',
        description='Override reflectance_node debug flag (true/false).',
    )

    reflectance_params_file_arg = DeclareLaunchArgument(
        'reflectance_params_file',
        default_value=default_reflectance_params,
        description='YAML params file for reflectance_node (leave empty to use defaults).',
    )

    static_tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mapir_optical_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
        arguments=[
            LaunchConfiguration('static_tf_x'),
            LaunchConfiguration('static_tf_y'),
            LaunchConfiguration('static_tf_z'),
            LaunchConfiguration('static_tf_roll'),
            LaunchConfiguration('static_tf_pitch'),
            LaunchConfiguration('static_tf_yaw'),
            LaunchConfiguration('static_tf_parent_frame'),
            LaunchConfiguration('static_tf_child_frame'),
        ],
    )

    def _as_bool(value: str) -> bool:
        return value.strip().lower() in ('1', 'true', 'yes', 'on')

    def _build_camera_node(context, *args, **kwargs):
        impl = LaunchConfiguration('camera_impl').perform(context).strip().lower()
        if impl in ('cpp', 'c++', 'cxx'):
            package = 'mapir_camera_ros2'
            executable = 'camera_node_cpp'
        else:
            package = 'mapir_camera_ros2'
            executable = 'camera_node'
        params = []
        camera_params_file = LaunchConfiguration('camera_params_file').perform(context).strip()
        if camera_params_file:
            params.append(camera_params_file)
        camera_info_url = LaunchConfiguration('camera_info_url').perform(context).strip()
        if camera_info_url:
            params.append({'camera_info_url': camera_info_url})
        return [
            Node(
                package=package,
                executable=executable,
                namespace=LaunchConfiguration('namespace'),
                name='camera',
                output='screen',
                parameters=params,
            )
        ]

    def _extract_indices_from_file(path: str) -> list[str]:
        if not path or not os.path.exists(path):
            return []
        indices: list[str] = []
        try:
            with open(path, encoding='utf-8') as handle:
                for line in handle:
                    match = re.search(r'^\s*indices:\s*\[(.*)\]\s*$', line)
                    if match:
                        raw = match.group(1)
                        for item in raw.split(','):
                            name = item.strip()
                            if name:
                                indices.append(name)
                        break
        except OSError:
            return []
        return indices

    def _build_indices_nodes(context, *args, **kwargs):
        if not _as_bool(LaunchConfiguration('enable_indices').perform(context)):
            return []

        per_node = _as_bool(LaunchConfiguration('indices_per_node').perform(context))
        use_all = _as_bool(LaunchConfiguration('indices_all').perform(context))
        params_file = LaunchConfiguration('indices_params_file').perform(context)

        if use_all:
            indices_list = sorted(supported_spectral_indices())
        else:
            indices_list = _extract_indices_from_file(params_file)

        params_list = []
        if params_file and os.path.exists(params_file):
            params_list.append(params_file)

        if not per_node:
            params = params_list
            return [
                Node(
                    package='mapir_camera_ros2',
                    executable='indices_node',
                    namespace=LaunchConfiguration('namespace'),
                    name='indices',
                    output='screen',
                    parameters=params,
                )
            ]

        if not indices_list:
            return [
                LogInfo(
                    msg=(
                        'indices_per_node enabled but no indices list found; '
                        'no indices nodes launched.'
                    )
                )
            ]

        nodes = []
        for index_name in indices_list:
            safe_name = f'indices_{index_name}'.replace('/', '_')
            params = params_list + [{'indices': [index_name]}]
            nodes.append(
                Node(
                    package='mapir_camera_ros2',
                    executable='indices_node',
                    namespace=LaunchConfiguration('namespace'),
                    name=safe_name,
                    output='screen',
                    parameters=params,
                )
            )
        return nodes

    def _build_reflectance_node(context, *args, **kwargs):
        if not _as_bool(LaunchConfiguration('enable_reflectance').perform(context)):
            return []

        params_file = LaunchConfiguration('reflectance_params_file').perform(context)
        params_list = []
        if params_file and os.path.exists(params_file):
            params_list.append(params_file)
        reflectance_debug = LaunchConfiguration('reflectance_debug').perform(context).strip()
        if reflectance_debug:
            params_list.append({'debug': _as_bool(reflectance_debug)})

        return [
            Node(
                package='mapir_camera_ros2',
                executable='reflectance_node',
                namespace=LaunchConfiguration('namespace'),
                name='reflectance',
                output='screen',
                parameters=params_list,
            )
        ]

    return LaunchDescription([
        namespace_arg,
        camera_params_file_arg,
        camera_info_url_arg,
        camera_impl_arg,
        publish_static_tf_arg,
        static_tf_parent_arg,
        static_tf_child_arg,
        static_tf_x_arg,
        static_tf_y_arg,
        static_tf_z_arg,
        static_tf_roll_arg,
        static_tf_pitch_arg,
        static_tf_yaw_arg,
        enable_indices_arg,
        indices_params_file_arg,
        indices_per_node_arg,
        indices_all_arg,
        enable_reflectance_arg,
        reflectance_debug_arg,
        reflectance_params_file_arg,
        static_tf_optical,
        OpaqueFunction(function=_build_camera_node),
        OpaqueFunction(function=_build_indices_nodes),
        OpaqueFunction(function=_build_reflectance_node),
    ])
