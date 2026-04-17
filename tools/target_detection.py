#!/usr/bin/env python3
"""ArUco marker detection and board pose estimation for reflectance calibration."""

from __future__ import annotations

from typing import NamedTuple
from pathlib import Path
import json

import cv2
import numpy as np
import yaml


class ArUcoDetection(NamedTuple):
    """ArUco marker detection result."""
    marker_id: int
    marker_corners_px: np.ndarray  # shape (4, 2), float32
    marker_center_px: tuple[float, float]


class TargetPose(NamedTuple):
    """3D board pose from PnP estimation."""
    rvec: np.ndarray  # Rotation vector (3,)
    tvec: np.ndarray  # Translation vector (3,)
    reprojection_error: float  # Reprojection error in pixels


class PanelROI(NamedTuple):
    """Projected panel region of interest."""
    name: str
    center_px: tuple[float, float]
    bounding_box: tuple[int, int, int, int]  # (x_min, y_min, x_max, y_max)


def detect_aruco_marker(
    image: np.ndarray,
    dictionary_name: str = '4X4_250',
    marker_id: int | None = None,
) -> ArUcoDetection | None:
    """
    Detect ArUco marker in image.

    Args:
        image: BGR image (uint8 or uint16)
        dictionary_name: ArUco dictionary name (e.g., '4X4_250', '5X5_250')
        marker_id: Specific marker ID to find (None to find any)

    Returns:
        ArUcoDetection if found, None otherwise
    """
    # Convert to uint8 if needed
    if image.dtype == np.uint16:
        img_min = image.min()
        img_max = image.max()
        if img_max > img_min:
            image = ((image.astype(np.float32) - img_min) / (img_max - img_min) * 255).astype(np.uint8)
        else:
            image = (image / 256).astype(np.uint8)

    # Convert to grayscale if color
    if len(image.shape) == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Get ArUco dictionary
    try:
        dict_id = getattr(cv2.aruco, f'DICT_{dictionary_name}')
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        # Fallback for OpenCV 4.6
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, f'DICT_{dictionary_name}')
        )

    # Detect markers
    try:
        # New API (OpenCV 4.7+)
        detector = cv2.aruco.ArucoDetector(aruco_dict)
        corners, ids, rejected = detector.detectMarkers(image)
    except (AttributeError, TypeError):
        # Old API fallback
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict)

    if ids is None or len(ids) == 0:
        return None

    # Find specific marker or first one
    if marker_id is not None:
        for i, detected_id in enumerate(ids.flatten()):
            if detected_id == marker_id:
                corner = corners[i][0].astype(np.float32)
                center = corner.mean(axis=0)
                return ArUcoDetection(
                    marker_id=int(detected_id),
                    marker_corners_px=corner,
                    marker_center_px=tuple(center),
                )
        return None
    else:
        # Return first marker found
        corner = corners[0][0].astype(np.float32)
        center = corner.mean(axis=0)
        return ArUcoDetection(
            marker_id=int(ids[0][0]),
            marker_corners_px=corner,
            marker_center_px=tuple(center),
        )


def estimate_board_pose(
    aruco_detection: ArUcoDetection,
    marker_size_mm: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> TargetPose:
    """
    Estimate 3D board pose from ArUco marker detection.

    Uses PnP (Perspective-n-Point) to estimate the 3D rotation and translation
    of the board from 4 marker corner points.

    Args:
        aruco_detection: Detected ArUco marker with corner coordinates
        marker_size_mm: Physical size of marker in mm
        camera_matrix: Camera intrinsic matrix (3x3)
        dist_coeffs: Lens distortion coefficients (ignored, using undistorted model)

    Returns:
        TargetPose with rotation vector, translation vector, and reprojection error
    """
    # Define 3D marker corners in board coordinates (mm)
    # Marker is centered at origin, size is marker_size_mm
    half_size = marker_size_mm / 2
    marker_3d = np.array([
        [-half_size, -half_size, 0],  # corner 0: top-left
        [half_size, -half_size, 0],   # corner 1: top-right
        [half_size, half_size, 0],    # corner 2: bottom-right
        [-half_size, half_size, 0],   # corner 3: bottom-left
    ], dtype=np.float32)

    # Get 2D marker corners and reorder if needed
    image_points = aruco_detection.marker_corners_px.copy()

    # OpenCV ArUco detector may return corners in different order
    # Reorder to: top-left, top-right, bottom-right, bottom-left
    # by sorting by Y then X
    corners_with_idx = [(image_points[i], i) for i in range(4)]

    # Sort by Y coordinate first (top vs bottom)
    corners_with_idx_sorted_y = sorted(corners_with_idx, key=lambda x: x[0][1])

    # Top two points (smallest Y)
    top_corners = corners_with_idx_sorted_y[:2]
    # Sort by X coordinate
    top_corners = sorted(top_corners, key=lambda x: x[0][0])

    # Bottom two points (largest Y)
    bottom_corners = corners_with_idx_sorted_y[2:]
    # Sort by X coordinate
    bottom_corners = sorted(bottom_corners, key=lambda x: x[0][0])

    # Reorder: TL, TR, BR, BL
    image_points_ordered = np.array([
        top_corners[0][0],      # top-left
        top_corners[1][0],      # top-right
        bottom_corners[1][0],   # bottom-right
        bottom_corners[0][0],   # bottom-left
    ], dtype=np.float32)

    # Solve PnP WITHOUT distortion (use zero distortion coefficients)
    # Distortion can cause errors when marker is not centered in image
    zero_dist_coeffs = np.zeros(5, dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(
        marker_3d,
        image_points_ordered,
        camera_matrix,
        zero_dist_coeffs,  # No distortion
        useExtrinsicGuess=False,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success:
        raise RuntimeError("PnP pose estimation failed")

    # Calculate reprojection error with zero distortion
    reprojected, _ = cv2.projectPoints(
        marker_3d,
        rvec,
        tvec,
        camera_matrix,
        zero_dist_coeffs,  # No distortion
    )
    reprojection_error = np.mean(
        np.linalg.norm(image_points_ordered - reprojected.squeeze(), axis=1)
    )

    return TargetPose(
        rvec=rvec.squeeze(),
        tvec=tvec.squeeze(),
        reprojection_error=float(reprojection_error),
    )


def project_panel_rois(
    panels_3d: list[dict],
    pose: TargetPose,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_shape: tuple[int, int],
) -> list[PanelROI]:
    """
    Project 3D panel coordinates to 2D image space.

    Args:
        panels_3d: List of panel dicts with 'name', 'center_mm', 'size_mm' keys
        pose: Estimated board pose (rvec, tvec)
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Lens distortion coefficients (ignored, using undistorted model)
        image_shape: Image shape (height, width)

    Returns:
        List of PanelROI with projected bounding boxes
    """
    panel_rois = []
    # Use zero distortion for consistency with pose estimation
    zero_dist_coeffs = np.zeros(5, dtype=np.float32)

    for panel in panels_3d:
        name = panel['name']
        center_mm = np.array(panel['center_mm'], dtype=np.float32)
        size_mm = np.array(panel['size_mm'], dtype=np.float32)

        # 3D panel corners in board coordinates
        half_w = size_mm[0] / 2
        half_h = size_mm[1] / 2
        panel_3d = np.array([
            center_mm + [-half_w, -half_h],
            center_mm + [half_w, -half_h],
            center_mm + [half_w, half_h],
            center_mm + [-half_w, half_h],
        ], dtype=np.float32)
        panel_3d = np.column_stack([panel_3d, np.zeros(4)])  # Add Z=0

        # Project to image without distortion
        points_2d, _ = cv2.projectPoints(
            panel_3d,
            pose.rvec,
            pose.tvec,
            camera_matrix,
            zero_dist_coeffs,  # No distortion
        )
        points_2d = points_2d.squeeze()

        # Get bounding box
        x_coords = points_2d[:, 0]
        y_coords = points_2d[:, 1]
        x_min = max(0, int(np.floor(x_coords.min())))
        x_max = min(image_shape[1], int(np.ceil(x_coords.max())))
        y_min = max(0, int(np.floor(y_coords.min())))
        y_max = min(image_shape[0], int(np.ceil(y_coords.max())))

        center_2d = (
            (x_min + x_max) / 2,
            (y_min + y_max) / 2,
        )

        panel_rois.append(PanelROI(
            name=name,
            center_px=center_2d,
            bounding_box=(x_min, y_min, x_max, y_max),
        ))

    return panel_rois


def detect_target_with_aruco(
    image: np.ndarray,
    target_layout_path: Path,
    camera_intrinsics_path: Path,
    dictionary_name: str = '4X4_250',
    marker_id: int | None = None,
) -> tuple[list[PanelROI], dict] | tuple[None, None]:
    """
    Complete ArUco marker detection and panel ROI projection pipeline.

    Args:
        image: Calibration image (BGR, uint8 or uint16)
        target_layout_path: Path to target_layout.json
        camera_intrinsics_path: Path to camera intrinsics YAML
        dictionary_name: ArUco dictionary name
        marker_id: Specific marker ID to find

    Returns:
        Tuple of (panel_rois, metadata) or (None, None) if detection failed
    """
    # Load configurations
    target_layout = load_target_layout(target_layout_path)
    camera_matrix, dist_coeffs = load_camera_intrinsics_from_yaml(camera_intrinsics_path)

    # Detect marker
    aruco_detection = detect_aruco_marker(image, dictionary_name, marker_id)
    if aruco_detection is None:
        return None, None

    # Get marker size from config
    aruco_config = target_layout.get('aruco_marker', {})
    marker_size_mm = aruco_config.get('size_mm', 40.0)

    # Estimate pose
    pose = estimate_board_pose(aruco_detection, marker_size_mm, camera_matrix, dist_coeffs)

    # Project panels
    panels_3d = target_layout.get('panels_in_board_coordinates', [])
    panel_rois = project_panel_rois(panels_3d, pose, camera_matrix, dist_coeffs, image.shape[:2])

    metadata = {
        'method': 'aruco_auto',
        'marker_id': aruco_detection.marker_id,
        'marker_corners_px': aruco_detection.marker_corners_px.tolist(),
        'board_pose_reprojection_error_px': pose.reprojection_error,
        'detected_panel_count': len(panel_rois),
    }

    return panel_rois, metadata


def load_target_layout(target_layout_path: Path) -> dict:
    """Load target geometry configuration from JSON."""
    with open(target_layout_path) as f:
        return json.load(f)


def load_camera_intrinsics_from_yaml(yaml_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera matrix and distortion coefficients from ROS camera calibration YAML.

    Args:
        yaml_path: Path to camera_info.yaml

    Returns:
        Tuple of (camera_matrix, distortion_coefficients)
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    # Extract camera matrix
    camera_matrix = np.array(data['camera_matrix']['data']).reshape(3, 3)

    # Extract distortion coefficients
    dist_coeffs = np.array(data['distortion_coefficients']['data'])

    return camera_matrix, dist_coeffs


# Re-export for compatibility with old QR code naming
QRDetection = ArUcoDetection
detect_qr_code = detect_aruco_marker

