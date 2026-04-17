"""Capture, calibration, and vignette helpers for the MAPIR Survey3 ROS 2 node.

The node itself is kept focused on ROS orchestration.  This module owns the
OpenCV/V4L2/GStreamer capture setup path plus the small pieces of camera-info
and vignette handling that belong directly to frame acquisition.
"""

from __future__ import annotations

from camera_info_manager.camera_info_manager import CameraInfoMissingError
from mapir_camera_core import (
    apply_vignette_correction,
    configure_v4l2_capture,
    load_vignette_images,
    open_v4l2_capture,
)
from sensor_msgs.msg import CameraInfo
import cv2


def open_camera(node, video_device: str) -> cv2.VideoCapture:
    """Open the image stream using either V4L2 or the configured GStreamer path."""
    if not node.use_gstreamer:
        return open_v4l2_capture(video_device)

    pipeline = build_gstreamer_pipeline(node)
    if node.debug:
        node.get_logger().info(f"GStreamer pipeline: {pipeline}")
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


def configure_camera(node) -> None:
    """Negotiate capture settings and cache the resulting frame dimensions."""
    if node.use_gstreamer:
        node.width = node.req_width
        node.height = node.req_height
        node.get_logger().info("GStreamer capture enabled; skipping V4L2 negotiation")
        return

    if node.pixel_format not in ("MJPG", "H264"):
        node.get_logger().warn(
            f"Unsupported pixel_format={node.pixel_format!r}, forcing MJPG"
        )
        node.pixel_format = "MJPG"

    negotiation = configure_v4l2_capture(
        node.cap,
        req_width=node.req_width,
        req_height=node.req_height,
        req_fps=node.req_fps,
        pixel_format=node.pixel_format,
    )

    node.width = negotiation.width if negotiation.width > 0 else node.req_width
    node.height = negotiation.height if negotiation.height > 0 else node.req_height

    node.get_logger().info(
        f"Negotiated capture: {negotiation.width}x{negotiation.height} "
        f"@ {negotiation.fps:.2f} Hz, "
        f"FOURCC={negotiation.fourcc_str!r}"
    )

    if node.debug and negotiation.backend_id is not None:
        node.get_logger().info(f"OpenCV backend id: {negotiation.backend_id}")


def build_gstreamer_pipeline(node) -> str:
    """Build the default USB-camera GStreamer pipeline for the active format."""
    custom = node.gstreamer_pipeline.strip()
    if custom:
        return custom

    device = node.video_device
    width = node.req_width
    height = node.req_height
    fps = int(round(node.req_fps))

    if node.pixel_format == "H264":
        return (
            f"v4l2src device={device} ! "
            f"video/x-h264,width={width},height={height},framerate={fps}/1 ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

    return (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def update_dimensions_from_frame(node, frame) -> None:
    """Update cached frame dimensions from an already captured image."""
    try:
        height, width = frame.shape[:2]
    except Exception:
        return
    if width > 0 and height > 0:
        node.width = int(width)
        node.height = int(height)


def default_camerainfo(node) -> CameraInfo:
    """Create an uncalibrated :class:`sensor_msgs.msg.CameraInfo` fallback."""
    msg = CameraInfo()
    msg.width = int(node.width)
    msg.height = int(node.height)
    return msg


def get_camerainfo_safe(node) -> CameraInfo:
    """Return calibrated camera info when available, else a size-only fallback."""
    try:
        return node.cinfo_manager.getCameraInfo()
    except CameraInfoMissingError:
        if node.debug:
            node.get_logger().info("CameraInfo missing; using default CameraInfo()")
        return default_camerainfo(node)
    except Exception as ex:
        node.get_logger().warn(
            f"getCameraInfo() failed; using default CameraInfo. Reason: {ex}"
        )
        return default_camerainfo(node)


def configure_vignette_if_enabled(node) -> None:
    """Load and validate flat fields before runtime vignette correction begins."""
    if not node.vignette_enabled:
        return

    paths = [
        node.vignette_flatfield_b_path.strip(),
        node.vignette_flatfield_g_path.strip(),
        node.vignette_flatfield_r_path.strip(),
    ]
    if not all(paths):
        node.get_logger().warn(
            "vignette_enabled=true but flat-field paths are incomplete"
        )
        node.get_logger().warn("Disabling vignette correction")
        node.vignette_enabled = False
        return

    try:
        flat_fields = load_vignette_images(paths)
    except Exception as ex:
        node.get_logger().warn(f"Failed to load vignette flat-fields: {ex}")
        node.get_logger().warn("Disabling vignette correction")
        node.vignette_enabled = False
        return

    expected_shape = (int(node.height), int(node.width))
    for idx, flat_field in enumerate(flat_fields):
        if flat_field.shape != expected_shape:
            node.get_logger().warn(
                f"Flat-field {idx} shape={flat_field.shape} does not match "
                f"camera frame shape={expected_shape}"
            )
            node.get_logger().warn("Disabling vignette correction")
            node.vignette_enabled = False
            return

    node._flat_fields_bgr = flat_fields
    node.get_logger().info("Vignette correction enabled")


def apply_runtime_vignette_correction(node, frame):
    """Apply vignette correction to one frame and fail closed on runtime errors."""
    if not node.vignette_enabled or node._flat_fields_bgr is None:
        return frame

    try:
        return apply_vignette_correction(
            frame,
            node._flat_fields_bgr,
            dark_current=node.vignette_dark_current,
        )
    except Exception as ex:
        node.get_logger().warn(
            f"Vignette correction failed: {ex}. Disabling vignette correction at runtime."
        )
        node.vignette_enabled = False
        return frame
