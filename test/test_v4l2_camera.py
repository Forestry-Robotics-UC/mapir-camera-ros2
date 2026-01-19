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
import cv2
import pytest

from mapir_camera_ros2.core import configure_v4l2_capture, fourcc_to_str


class FakeCapture:
    def __init__(self, values=None):
        self._values = values or {}

    def set(self, prop, value):  # noqa: A003 - mirrors cv2.VideoCapture API
        self._values[prop] = value
        return True

    def get(self, prop):
        return self._values.get(prop, 0)


def test_fourcc_to_str_round_trip():
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    assert fourcc_to_str(fourcc) == 'MJPG'


def test_fourcc_to_str_invalid_returns_placeholder():
    assert fourcc_to_str(None) == '????'


def test_configure_v4l2_capture_rejects_invalid_pixel_format():
    cap = FakeCapture()
    with pytest.raises(ValueError):
        configure_v4l2_capture(cap, req_width=1, req_height=1, req_fps=1.0, pixel_format='MJ')


def test_configure_v4l2_capture_returns_negotiated_values():
    cap = FakeCapture(
        {
            cv2.CAP_PROP_FRAME_WIDTH: 640,
            cv2.CAP_PROP_FRAME_HEIGHT: 480,
            cv2.CAP_PROP_FPS: 30.0,
            cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG'),
            cv2.CAP_PROP_BACKEND: int(cv2.CAP_V4L2),
        }
    )
    negotiation = configure_v4l2_capture(
        cap,
        req_width=640,
        req_height=480,
        req_fps=30.0,
        pixel_format='MJPG',
    )
    assert negotiation.width == 640
    assert negotiation.height == 480
    assert negotiation.fps == 30.0
    assert negotiation.fourcc_str == 'MJPG'
    assert negotiation.backend_id == int(cv2.CAP_V4L2)


def test_configure_v4l2_capture_backend_none_on_error():
    class ErrorCapture(FakeCapture):
        def get(self, prop):
            if prop == cv2.CAP_PROP_BACKEND:
                raise RuntimeError('backend error')
            return super().get(prop)

    cap = ErrorCapture(
        {
            cv2.CAP_PROP_FRAME_WIDTH: 320,
            cv2.CAP_PROP_FRAME_HEIGHT: 240,
            cv2.CAP_PROP_FPS: 15.0,
            cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG'),
        }
    )
    negotiation = configure_v4l2_capture(
        cap,
        req_width=320,
        req_height=240,
        req_fps=15.0,
        pixel_format='MJPG',
    )
    assert negotiation.backend_id is None
