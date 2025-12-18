from __future__ import annotations

from dataclasses import dataclass
import re

import cv2


def fourcc_to_str(fourcc_int: int) -> str:
    """Convert OpenCV FOURCC int to a readable 4-character code."""
    try:
        return ''.join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
    except Exception:
        return '????'


@dataclass(frozen=True)
class V4L2Negotiation:
    """Negotiated capture properties after attempting to configure V4L2/OpenCV."""

    width: int
    height: int
    fps: float
    fourcc_int: int
    fourcc_str: str
    backend_id: int | None


def open_v4l2_capture(video_device: str) -> cv2.VideoCapture:
    """
    Open a camera using OpenCV's V4L2 backend.

    Accepts:
      - numeric index (e.g., '0')
      - /dev/videoX
      - any path OpenCV understands
    """
    if video_device.isdigit():
        return cv2.VideoCapture(int(video_device), cv2.CAP_V4L2)

    match = re.match(r'^/dev/video(\d+)$', video_device)
    if match:
        return cv2.VideoCapture(int(match.group(1)), cv2.CAP_V4L2)

    return cv2.VideoCapture(video_device, cv2.CAP_V4L2)


def configure_v4l2_capture(
    cap: cv2.VideoCapture,
    *,
    req_width: int,
    req_height: int,
    req_fps: float,
    pixel_format: str,
) -> V4L2Negotiation:
    """
    Attempt to configure pixel format, resolution, and FPS.

    Returns the negotiated settings as a `V4L2Negotiation`.
    """
    fmt = str(pixel_format).strip().upper()
    if len(fmt) != 4:
        raise ValueError(f'pixel_format must be a 4-char FOURCC string (got {pixel_format!r})')

    fourcc = cv2.VideoWriter_fourcc(*fmt)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(req_width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(req_height))
    cap.set(cv2.CAP_PROP_FPS, float(req_fps))

    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fps = float(cap.get(cv2.CAP_PROP_FPS))
    got_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))

    backend_id: int | None
    try:
        backend_id = int(cap.get(cv2.CAP_PROP_BACKEND))
    except Exception:
        backend_id = None

    return V4L2Negotiation(
        width=got_w,
        height=got_h,
        fps=got_fps,
        fourcc_int=got_fourcc,
        fourcc_str=fourcc_to_str(got_fourcc),
        backend_id=backend_id,
    )
