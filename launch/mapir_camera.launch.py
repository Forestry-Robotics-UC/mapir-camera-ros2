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
        default_value='',
        description='Optional override for V4L2 device path or numeric index.',
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='',
        description='Optional override for requested image width (pixels).',
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='',
        description='Optional override for requested image height (pixels).',
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='',
        description='Optional override for target frame rate (Hz).',
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='',
        description='Optional override for pixel format: MJPG or H264.',
    )
    use_gstreamer_arg = DeclareLaunchArgument(
        'use_gstreamer',
        default_value='',
        description='Optional override to use GStreamer pipeline for capture.',
    )
    gstreamer_pipeline_arg = DeclareLaunchArgument(
        'gstreamer_pipeline',
        default_value='',
        description='Custom GStreamer pipeline string (optional).',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='',
        description='Optional override for frame_id for Image and CameraInfo.',
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='',
        description='Optional override for camera name for camera_info_manager.',
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description=(
            'Optional override for camera calibration URL (e.g., file:///abs/path/to/calib.yaml). '
            'If empty, value comes from params file or node default.'
        ),
    )

    qos_best_effort_arg = DeclareLaunchArgument(
        'qos_best_effort',
        default_value='',
        description='Optional override for QoS reliability (BEST_EFFORT when true).',
    )

    qos_depth_arg = DeclareLaunchArgument(
        'qos_depth',
        default_value='',
        description='Optional override for publisher queue depth.',
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable extra debug logging in the camera node.',
    )

    debug_period_s_arg = DeclareLaunchArgument(
        'debug_period_s',
        default_value='',
        description='Optional override for debug throttle period (seconds).',
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

    def _as_bool(value: str) -> bool:
        return value.strip().lower() in ('1', 'true', 'yes', 'on')

    def _add_if_set(
        context,
        *,
        launch_arg: str,
        param_name: str,
        caster,
        out_params: dict,
        out_sources: dict,
    ) -> None:
        raw = LaunchConfiguration(launch_arg).perform(context).strip()
        if raw == '':
            return
        out_params[param_name] = caster(raw)
        out_sources[param_name] = f'launch:{launch_arg}'

    def _build_camera_node(context, *args, **kwargs):
        impl = LaunchConfiguration('camera_impl').perform(context).strip().lower()
        if impl in ('cpp', 'c++', 'cxx'):
            package = 'mapir_camera_ros2'
            executable = 'camera_node_cpp'
        else:
            package = 'mapir_camera_ros2'
            executable = 'camera_node'

        camera_params_file = LaunchConfiguration('camera_params_file').perform(context).strip()
        if not camera_params_file:
            camera_params_file = default_camera_params_file

        launch_overrides = {
            'debug': _as_bool(LaunchConfiguration('debug').perform(context)),
        }
        launch_sources = {'debug': 'launch:debug'}

        _add_if_set(
            context,
            launch_arg='video_device',
            param_name='video_device',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='image_width',
            param_name='image_width',
            caster=int,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='image_height',
            param_name='image_height',
            caster=int,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='framerate',
            param_name='framerate',
            caster=float,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='pixel_format',
            param_name='pixel_format',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='use_gstreamer',
            param_name='use_gstreamer',
            caster=_as_bool,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='gstreamer_pipeline',
            param_name='gstreamer_pipeline',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='frame_id',
            param_name='frame_id',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='camera_name',
            param_name='camera_name',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='camera_info_url',
            param_name='camera_info_url',
            caster=str,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='qos_best_effort',
            param_name='qos_best_effort',
            caster=_as_bool,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='qos_depth',
            param_name='qos_depth',
            caster=int,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )
        _add_if_set(
            context,
            launch_arg='debug_period_s',
            param_name='debug_period_s',
            caster=float,
            out_params=launch_overrides,
            out_sources=launch_sources,
        )

        source_summary = ', '.join(
            f'{name}={source}' for name, source in sorted(launch_sources.items())
        )
        source_message = (
            '[mapir_camera.launch] parameter precedence: '
            'launch overrides > params file > node defaults. '
            f'params_file={camera_params_file}. '
            f'explicit_overrides={source_summary}'
        )

        camera_params = [camera_params_file, launch_overrides]
        return [
            LogInfo(msg=source_message, condition=IfCondition(LaunchConfiguration('debug'))),
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
                LogInfo(
                    msg=(
                        'indices_per_node enabled but no indices list found; '
                        'no indices nodes launched.'
                    )
                )
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
