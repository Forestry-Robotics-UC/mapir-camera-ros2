#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mapir_camera_core.spectral_indices import supported_spectral_indices


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('mapir_camera_ros2')

    default_camera_params_file = os.path.join(pkg_share, 'config', 'mapir_camera_params.yaml')

    default_calib_file = os.path.expanduser('~/.ros/camera_info/mapir3_ocn.yaml')
    default_calib_url = (
        f'file://{default_calib_file}' if os.path.exists(default_calib_file) else ''
    )

    default_indices_params_file = os.path.join(pkg_share, 'config', 'mapir_indices_params.yaml')
    default_indices_params = (
        default_indices_params_file if os.path.exists(default_indices_params_file) else ''
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='mapir',
        description='Namespace for camera node and topics (e.g., mapir).',
    )

    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=default_camera_params_file,
        description='Path to a ROS 2 params YAML file for camera_node.',
    )

    camera_impl_arg = DeclareLaunchArgument(
        'camera_impl',
        default_value='cpp',
        description='Camera implementation: "py" (camera_node) or "cpp" (camera_node_cpp).',
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device path or numeric index (e.g., /dev/video0 or 0).',
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Requested image width (pixels).',
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Requested image height (pixels).',
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Target frame rate (Hz).',
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='MJPG',
        description='Pixel format: MJPG or H264 (device-dependent).',
    )
    use_gstreamer_arg = DeclareLaunchArgument(
        'use_gstreamer',
        default_value='false',
        description='Use GStreamer pipeline for capture (optional).',
    )
    gstreamer_pipeline_arg = DeclareLaunchArgument(
        'gstreamer_pipeline',
        default_value='',
        description='Custom GStreamer pipeline string (optional).',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='mapir3_optical_frame',
        description='Frame ID for published Image and CameraInfo.',
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='mapir3_ocn',
        description='Camera name for camera_info_manager.',
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=default_calib_url,
        description=(
            'Camera calibration URL (e.g., file:///abs/path/to/calib.yaml). '
            'If empty, no calibration is loaded.'
        ),
    )

    qos_best_effort_arg = DeclareLaunchArgument(
        'qos_best_effort',
        default_value='true',
        description='Use BEST_EFFORT QoS for camera streams (recommended).',
    )

    qos_depth_arg = DeclareLaunchArgument(
        'qos_depth',
        default_value='5',
        description='Publisher queue depth.',
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable extra debug logging in the camera node.',
    )

    debug_period_s_arg = DeclareLaunchArgument(
        'debug_period_s',
        default_value='1.0',
        description='Throttle period for debug logs (seconds).',
    )

    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='true',
        description='Publish placeholder static TF mapir3_link -> mapir3_optical_frame.',
    )

    static_tf_parent_arg = DeclareLaunchArgument(
        'static_tf_parent_frame',
        default_value='mapir3_link',
        description='Static TF parent frame.',
    )

    static_tf_child_arg = DeclareLaunchArgument(
        'static_tf_child_frame',
        default_value='mapir3_optical_frame',
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

    indices_enabled_arg = DeclareLaunchArgument(
        'indices_enabled',
        default_value='true',
        description='Initial enabled state for indices_node (can be toggled at runtime).',
    )

    indices_params_file_arg = DeclareLaunchArgument(
        'indices_params_file',
        default_value=default_indices_params,
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

    camera_params = [
        LaunchConfiguration('camera_params_file'),
        {
            'video_device': ParameterValue(
                LaunchConfiguration('video_device'),
                value_type=str,
            ),
            'image_width': ParameterValue(
                LaunchConfiguration('image_width'),
                value_type=int,
            ),
            'image_height': ParameterValue(
                LaunchConfiguration('image_height'),
                value_type=int,
            ),
            'framerate': ParameterValue(
                LaunchConfiguration('framerate'),
                value_type=float,
            ),
            'pixel_format': ParameterValue(
                LaunchConfiguration('pixel_format'),
                value_type=str,
            ),
            'use_gstreamer': ParameterValue(
                LaunchConfiguration('use_gstreamer'),
                value_type=bool,
            ),
            'gstreamer_pipeline': ParameterValue(
                LaunchConfiguration('gstreamer_pipeline'),
                value_type=str,
            ),
            'frame_id': ParameterValue(
                LaunchConfiguration('frame_id'),
                value_type=str,
            ),
            'camera_name': ParameterValue(
                LaunchConfiguration('camera_name'),
                value_type=str,
            ),
            'camera_info_url': ParameterValue(
                LaunchConfiguration('camera_info_url'),
                value_type=str,
            ),
            'qos_best_effort': ParameterValue(
                LaunchConfiguration('qos_best_effort'),
                value_type=bool,
            ),
            'qos_depth': ParameterValue(
                LaunchConfiguration('qos_depth'),
                value_type=int,
            ),
            'debug': ParameterValue(
                LaunchConfiguration('debug'),
                value_type=bool,
            ),
            'debug_period_s': ParameterValue(
                LaunchConfiguration('debug_period_s'),
                value_type=float,
            ),
        },
    ]

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
        return [
            Node(
                package=package,
                executable=executable,
                namespace=LaunchConfiguration('namespace'),
                name='camera',
                output='screen',
                parameters=camera_params,
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

        common_params = {
            'enabled': ParameterValue(
                LaunchConfiguration('indices_enabled'),
                value_type=bool,
            ),
            'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            'qos_best_effort': ParameterValue(
                LaunchConfiguration('qos_best_effort'),
                value_type=bool,
            ),
            'qos_depth': ParameterValue(LaunchConfiguration('qos_depth'), value_type=int),
        }

        if not per_node:
            params = params_list + [common_params]
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
                LogInfo(msg='indices_per_node enabled but no indices list found; no indices nodes launched.')
            ]

        nodes = []
        for index_name in indices_list:
            safe_name = f"indices_{index_name}".replace('/', '_')
            params = params_list + [
                common_params,
                {
                    'indices': ParameterValue([index_name], value_type=list),
                },
            ]
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

    return LaunchDescription([
        namespace_arg,
        camera_params_file_arg,
        camera_impl_arg,
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        use_gstreamer_arg,
        gstreamer_pipeline_arg,
        frame_id_arg,
        camera_name_arg,
        camera_info_url_arg,
        qos_best_effort_arg,
        qos_depth_arg,
        debug_arg,
        debug_period_s_arg,
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
        indices_enabled_arg,
        indices_params_file_arg,
        indices_per_node_arg,
        indices_all_arg,
        static_tf_optical,
        OpaqueFunction(function=_build_camera_node),
        OpaqueFunction(function=_build_indices_nodes),
    ])
