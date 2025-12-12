#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: mapir_survey3
#
# Description:
#   Launch MAPIR Survey3 camera node with proper frame_id conventions.
#   Publishes under /mapir (e.g., /mapir/image_raw and /mapir/camera_info).
#
# Notes (REP-105 / optical frame conventions):
#   Optical frame uses:
#     +X to the right in the image
#     +Y down in the image
#     +Z forward (out of the camera)
#   If you do not yet have a URDF, a temporary static transform from mapir3_link
#   to mapir3_optical_frame is a practical placeholder. :contentReference[oaicite:1]{index=1}

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------
    # Launch arguments (user-exposed)
    # -----------------------
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable extra debug logging in the camera node.'
    )

    qos_best_effort_arg = DeclareLaunchArgument(
        'qos_best_effort',
        default_value='true',
        description=(
            'Publish image/camera_info with BEST_EFFORT QoS (recommended for camera streams). '
            'Set to false to use RELIABLE.'
        )
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description=(
            'Camera calibration URL (e.g., file:///abs/path/to/calib.yaml). '
            'If empty, the launch file will try to use the package config default if present.'
        )
    )

    # -----------------------
    # Default calibration resolution (package-provided)
    # -----------------------
    pkg_share = get_package_share_directory('mapir_camera_ros2')
    default_calib_file = os.path.join(pkg_share, 'config', 'mapir3_ocn.yaml')
    default_calib_url = f'file://{default_calib_file}' if os.path.exists(default_calib_file) else ''

    # If user did not supply camera_info_url, fall back to package default (if available).
    # LaunchConfiguration is always a string; we pass both and let the node load if valid.
    user_calib_url = LaunchConfiguration('camera_info_url')

    # -----------------------
    # Static TF: mapir3_link -> mapir3_optical_frame (placeholder until URDF)
    # -----------------------
    static_tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mapir_optical_tf',
        output='screen',
        arguments=[
            '0', '0', '0',                 # translation (m)
            '1.5707963', '0', '3.1415926', # roll, pitch, yaw (rad) for optical frame convention
            'mapir3_link',                 # parent frame
            'mapir3_optical_frame'         # child frame
        ]
    )


    camera_node = Node(
        package='mapir_camera_ros2',
        executable='camera_node',
        namespace='mapir',
        name='camera',
        output='screen',
        parameters=[{
            # Device / capture settings
            'video_device': '/dev/video0',
            'image_width': 1920,
            'image_height': 1440,
            'framerate': 60.0,
            'pixel_format': 'MJPG',

            # Frames / naming (REP-105)
            'frame_id': 'mapir3_optical_frame',
            'camera_name': 'mapir3_ocn',

            # User-exposed controls
            'debug': LaunchConfiguration('debug'),
            'qos_best_effort': LaunchConfiguration('qos_best_effort'),
            'camera_info_url': user_calib_url if str(user_calib_url) else default_calib_url,
        }],
    )

    return LaunchDescription([
        debug_arg,
        qos_best_effort_arg,
        camera_info_url_arg,
        static_tf_optical,
        camera_node,
    ])
