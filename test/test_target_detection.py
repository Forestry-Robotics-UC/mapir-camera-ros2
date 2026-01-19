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
import numpy as np
import pytest

from mapir_camera_ros2.core import (
    aruco_dictionary_from_name,
    compute_panel_quad_from_fiducial,
    detect_aruco_fiducial,
    inset_quad,
    quad_area,
    roi_to_quad,
    split_panel_quad,
)


def _stub_aruco(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    class DummyAruco:
        DICT_4X4_50 = 42

        def getPredefinedDictionary(self, dict_id):
            return {'id': dict_id}

    dummy = DummyAruco()
    dummy.ArucoDetector = object()
    monkeypatch.setattr(td.cv2, 'aruco', dummy, raising=False)
    return dummy


def test_roi_to_quad():
    quad = roi_to_quad([0, 0, 2, 1])
    assert quad.shape == (4, 2)
    assert np.allclose(quad[0], [0.0, 0.0])
    assert np.allclose(quad[2], [2.0, 1.0])


def test_roi_to_quad_invalid_length():
    with pytest.raises(ValueError):
        roi_to_quad([0, 0, 1])


def test_quad_area_square():
    quad = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    assert quad_area(quad) == pytest.approx(1.0)


def test_compute_panel_quad_from_fiducial_right():
    fiducial = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    panel = compute_panel_quad_from_fiducial(
        fiducial,
        side='right',
        panel_scale_w=1.0,
        panel_scale_h=1.0,
        panel_gap_scale=0.0,
    )
    expected = np.array([[1, 0], [2, 0], [2, 1], [1, 1]], dtype=np.float32)
    assert np.allclose(panel, expected)


def test_compute_panel_quad_from_fiducial_invalid_side():
    fiducial = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    with pytest.raises(ValueError):
        compute_panel_quad_from_fiducial(
            fiducial,
            side='diagonal',
            panel_scale_w=1.0,
            panel_scale_h=1.0,
            panel_gap_scale=0.0,
        )


def test_inset_quad_reduces_size():
    quad = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
    inset = inset_quad(quad, 0.1)
    assert inset.shape == (4, 2)
    assert inset[0, 0] > quad[0, 0]
    assert inset[0, 1] > quad[0, 1]


def test_inset_quad_zero_ratio_no_change():
    quad = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
    inset = inset_quad(quad, 0.0)
    assert np.allclose(inset, quad)


def test_split_panel_quad_grid_size():
    panel = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], dtype=np.float32)
    patches = split_panel_quad(panel, grid=(2, 2), inset_ratio=0.0)
    assert len(patches) == 4
    assert patches[0].shape == (4, 2)


def test_aruco_dictionary_invalid_name(monkeypatch):
    _stub_aruco(monkeypatch)
    with pytest.raises(ValueError):
        aruco_dictionary_from_name('DICT_DOES_NOT_EXIST')


def test_detect_aruco_fiducial_round_trip(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    _stub_aruco(monkeypatch)

    corners = np.array(
        [[[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]]],
        dtype=np.float32,
    )
    ids = np.array([[0]], dtype=np.int32)

    class StubDetector:
        def detectMarkers(self, gray):
            return [corners], ids, None

    def fake_detect_markers(gray, dictionary, parameters=None):
        return [corners], ids, None

    monkeypatch.setattr(td, '_aruco_detector_from_name', lambda name: StubDetector())
    monkeypatch.setattr(td.cv2.aruco, 'ArucoDetector', object(), raising=False)
    monkeypatch.setattr(td.cv2.aruco, 'detectMarkers', fake_detect_markers, raising=False)

    image = np.zeros((40, 40, 3), dtype=np.uint8)
    found = detect_aruco_fiducial(
        image,
        dictionary_name='DICT_4X4_50',
        fiducial_id=0,
    )
    assert found is not None
    assert found.shape == (4, 2)


def test_detect_aruco_fiducial_no_ids_returns_none(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    _stub_aruco(monkeypatch)

    class StubDetector:
        def detectMarkers(self, gray):
            return [], None, None

    monkeypatch.setattr(td, '_aruco_detector_from_name', lambda name: StubDetector())

    image = np.zeros((10, 10, 3), dtype=np.uint8)
    found = detect_aruco_fiducial(
        image,
        dictionary_name='DICT_4X4_50',
        fiducial_id=0,
    )
    assert found is None


def test_detect_aruco_fiducial_id_not_found(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    _stub_aruco(monkeypatch)

    corners = np.array([[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]], dtype=np.float32)
    ids = np.array([[5]], dtype=np.int32)

    class StubDetector:
        def detectMarkers(self, gray):
            return [corners], ids, None

    monkeypatch.setattr(td, '_aruco_detector_from_name', lambda name: StubDetector())

    image = np.zeros((10, 10, 3), dtype=np.uint8)
    found = detect_aruco_fiducial(
        image,
        dictionary_name='DICT_4X4_50',
        fiducial_id=0,
    )
    assert found is None


def test_detect_aruco_fiducial_negative_id_selects_first(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    _stub_aruco(monkeypatch)

    first = np.array([[[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]], dtype=np.float32)
    second = np.array([[[5.0, 5.0], [7.0, 5.0], [7.0, 7.0], [5.0, 7.0]]], dtype=np.float32)
    ids = np.array([[10], [11]], dtype=np.int32)

    class StubDetector:
        def detectMarkers(self, gray):
            return [first, second], ids, None

    monkeypatch.setattr(td, '_aruco_detector_from_name', lambda name: StubDetector())

    image = np.zeros((10, 10), dtype=np.float32)
    found = detect_aruco_fiducial(
        image,
        dictionary_name='DICT_4X4_50',
        fiducial_id=-1,
    )
    assert found is not None
    assert np.allclose(found, first.reshape((4, 2)))


def test_detect_aruco_fiducial_min_area_filters(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    _stub_aruco(monkeypatch)

    corners = np.array([[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]], dtype=np.float32)
    ids = np.array([[0]], dtype=np.int32)

    class StubDetector:
        def detectMarkers(self, gray):
            return [corners], ids, None

    monkeypatch.setattr(td, '_aruco_detector_from_name', lambda name: StubDetector())

    image = np.zeros((10, 10), dtype=np.float32)
    found = detect_aruco_fiducial(
        image,
        dictionary_name='DICT_4X4_50',
        fiducial_id=0,
        min_area_px=5.0,
    )
    assert found is None


def test_aruco_dictionary_name_without_prefix(monkeypatch):
    _stub_aruco(monkeypatch)
    dictionary = aruco_dictionary_from_name('4X4_50')
    assert dictionary is not None


def test_aruco_detector_from_name_builds_detector(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    class DummyParams:
        pass

    class DummyDetector:
        def __init__(self, dictionary, params):
            self.dictionary = dictionary
            self.params = params

    class DummyAruco:
        DICT_4X4_50 = 7
        DetectorParameters = DummyParams
        ArucoDetector = DummyDetector

        def getPredefinedDictionary(self, dict_id):
            return {'id': dict_id}

    monkeypatch.setattr(td.cv2, 'aruco', DummyAruco(), raising=False)
    detector = td._aruco_detector_from_name('4X4_50')
    assert isinstance(detector, DummyDetector)


def test_aruco_params_create_fallback(monkeypatch):
    import mapir_camera_ros2.core.target_detection as td

    class DummyParams:
        pass

    class DummyAruco:
        def DetectorParameters_create(self):
            return DummyParams()

    monkeypatch.setattr(td.cv2, 'aruco', DummyAruco(), raising=False)
    params = td._aruco_params()
    assert isinstance(params, DummyParams)
