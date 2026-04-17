"""UVC control helpers for the MAPIR Survey3 ROS 2 node.

These helpers intentionally keep device-control side effects separate from the
main node constructor so the capture path remains readable and the optional
`v4l2-ctl` integration is easy to reason about in isolation.
"""

from __future__ import annotations

import shutil
import subprocess


def apply_uvc_controls_if_requested(node) -> None:
    """Apply requested UVC controls once and record the read-back state.

    Controls with negative values are treated as "leave unchanged".  When
    controls are applied successfully, the helper stores both the requested and
    observed values on the node so metadata publishing and debug logs can report
    the actual locked state.
    """
    if not node.uvc_controls_enabled:
        return

    if shutil.which("v4l2-ctl") is None:
        node.get_logger().warn("uvc_controls_enabled=true but v4l2-ctl not found")
        return

    controls_device = node.uvc_controls_device.strip() or node.video_device
    if not controls_device:
        node.get_logger().warn("No device available for UVC controls")
        return
    node._uvc_controls_device_applied = controls_device

    requested = {
        "auto_exposure": node.auto_exposure_mode,
        "exposure_time_absolute": node.exposure_time_absolute,
        "gain": node.gain,
        "exposure_dynamic_framerate": node.exposure_dynamic_framerate,
        "white_balance_automatic": node.white_balance_automatic,
        "white_balance_temperature": node.white_balance_temperature,
        "power_line_frequency": node.power_line_frequency,
    }
    requested = {key: value for key, value in requested.items() if int(value) >= 0}
    node._uvc_controls_requested = dict(requested)
    if not requested:
        node.get_logger().warn(
            "uvc_controls_enabled=true but no control values set (all < 0); skipping lock"
        )
        return

    set_arg = ",".join(f"{key}={value}" for key, value in requested.items())
    set_cmd = ["v4l2-ctl", "-d", controls_device, "-c", set_arg]
    set_proc = subprocess.run(
        set_cmd,
        capture_output=True,
        text=True,
        check=False,
    )
    if set_proc.returncode != 0:
        stderr = set_proc.stderr.strip()
        node.get_logger().warn(
            f"Failed to set UVC controls on {controls_device}: {stderr or 'unknown error'}"
        )
        return

    get_arg = ",".join(requested.keys())
    get_cmd = ["v4l2-ctl", "-d", controls_device, f"--get-ctrl={get_arg}"]
    get_proc = subprocess.run(
        get_cmd,
        capture_output=True,
        text=True,
        check=False,
    )
    if get_proc.returncode != 0:
        stderr = get_proc.stderr.strip()
        node.get_logger().warn(
            f"Failed to read back UVC controls on {controls_device}: {stderr or 'unknown error'}"
        )
        return

    applied: dict[str, str] = {}
    for line in get_proc.stdout.splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        applied[key.strip()] = value.strip()

    node._uvc_controls_applied = applied
    node._uvc_controls_locked = True
    node.get_logger().info(
        f"UVC controls locked on {controls_device}: {node._uvc_controls_applied}"
    )
