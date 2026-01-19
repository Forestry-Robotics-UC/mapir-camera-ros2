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
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _make_bgr_image(height: int = 2, width: int = 2) -> Image:
    data = np.zeros((height, width, 3), dtype=np.uint8)
    data[:, :, 0] = 10
    data[:, :, 1] = 20
    data[:, :, 2] = 30

    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = width * 3
    msg.data = data.tobytes()
    return msg


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package='mapir_camera_ros2',
        executable='reflectance_node',
        name='reflectance_test',
        parameters=[
            {
                'input_topic': 'image_raw',
                'output_topic': 'image_reflectance',
                'status_topic': 'reflectance/status',
                'enable': True,
                'detect_rate_hz': 0.0,
                'debug': False,
            }
        ],
        output='screen',
    )

    return LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestReflectanceNode(unittest.TestCase):
    def test_reflectance_publishes(self):
        rclpy.init()
        node = rclpy.create_node('reflectance_launch_test')
        try:
            qos_profile = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            pub = node.create_publisher(Image, 'image_raw', qos_profile)
            received = {}

            def _reflectance_cb(msg):
                received['reflectance'] = msg

            def _status_cb(msg):
                received['status'] = msg

            node.create_subscription(Image, 'image_reflectance', _reflectance_cb, qos_profile)
            node.create_subscription(String, 'reflectance/status', _status_cb, qos_profile)

            deadline = time.time() + 5.0
            while time.time() < deadline and (
                'reflectance' not in received or 'status' not in received
            ):
                pub.publish(_make_bgr_image())
                rclpy.spin_once(node, timeout_sec=0.1)

            self.assertIn('reflectance', received)
            self.assertIn('status', received)
            out = received['reflectance']
            self.assertEqual(out.encoding, '32FC3')
            self.assertEqual(out.height, 2)
            self.assertEqual(out.width, 2)
        finally:
            node.destroy_node()
            rclpy.shutdown()
