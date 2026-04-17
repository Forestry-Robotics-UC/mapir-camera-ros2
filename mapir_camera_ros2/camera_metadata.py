"""Metadata capture helpers for the MAPIR Survey3 ROS 2 node.

The Survey3 metadata path is separate from the image stream and relies on a
small background reader.  This module keeps the parsing, reader lifecycle, and
ROS publication logic together so the camera node can treat metadata as an
optional side channel.
"""

from __future__ import annotations

import json
import os
import shutil
import struct
import subprocess
import threading
import time

from std_msgs.msg import String


def parse_uvch_record(record: bytes):
    """Parse one 22-byte UVC metadata record into a structured dictionary.

    The returned mapping intentionally keeps the low-level field names so the
    published metadata stays close to the transport structure exposed by
    ``v4l2-ctl`` and Linux UVC metadata buffers.
    """
    if len(record) != 22:
        return None
    ns, sof, header_len, header_flags = struct.unpack_from("<QHBB", record, 0)
    if header_len < 2 or header_len > 12:
        return None

    payload = record[12:22]
    metadata = {
        "uvch_ns": int(ns),
        "uvch_sof": int(sof),
        "uvc_header_length": int(header_len),
        "uvc_header_flags": int(header_flags),
        "fid": int(header_flags & 0x01),
        "eof": int((header_flags >> 1) & 0x01),
        "pts_present": bool(header_flags & 0x04),
        "scr_present": bool(header_flags & 0x08),
        "eoh": bool(header_flags & 0x80),
    }

    if metadata["pts_present"] and len(payload) >= 6:
        metadata["uvc_pts"] = int(struct.unpack_from("<I", payload, 2)[0])
    if metadata["scr_present"] and len(payload) >= 12:
        metadata["uvc_scr_stc"] = int(struct.unpack_from("<I", payload, 6)[0])
        metadata["uvc_scr_sof"] = int(struct.unpack_from("<H", payload, 10)[0])
    return metadata


def start_metadata_reader(node) -> None:
    """Start the optional background process that streams metadata records."""
    if not node.metadata_topic:
        node.get_logger().warn(
            "metadata_enabled=true but metadata_topic is empty; disabling metadata publisher"
        )
        node.metadata_enabled = False
        return
    if shutil.which("v4l2-ctl") is None:
        node.get_logger().warn(
            "metadata_enabled=true but v4l2-ctl is not available in PATH. "
            "Continuing without metadata capture."
        )
        node.metadata_enabled = False
        return

    if node.metadata_log_path:
        try:
            log_dir = os.path.dirname(node.metadata_log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            node._metadata_log_file = open(node.metadata_log_path, "a", encoding="utf-8")
        except OSError as ex:
            node.get_logger().warn(
                f"Could not open metadata_log_path={node.metadata_log_path!r}: {ex}"
            )

    cmd = [
        "v4l2-ctl",
        "-d",
        node.metadata_device,
        "--stream-mmap",
        "--stream-to=-",
    ]
    try:
        node._metadata_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
    except OSError as ex:
        node.get_logger().warn(
            f"Could not start metadata stream command ({cmd}): {ex}. "
            "Continuing without metadata capture."
        )
        node.metadata_enabled = False
        return

    node._metadata_thread = threading.Thread(
        target=metadata_reader_loop,
        args=(node,),
        name="mapir_uvch_reader",
        daemon=True,
    )
    node._metadata_thread.start()
    node.get_logger().info(
        f"Metadata capture enabled: device={node.metadata_device}, topic={node.metadata_topic}"
    )


def metadata_reader_loop(node) -> None:
    """Continuously read fixed-size metadata records from the helper process."""
    if node._metadata_proc is None or node._metadata_proc.stdout is None:
        return
    stream = node._metadata_proc.stdout
    buffer = bytearray()
    try:
        while not node._stop_metadata.is_set():
            chunk = stream.read(4096)
            if not chunk:
                if node._metadata_proc.poll() is not None:
                    break
                time.sleep(0.002)
                continue
            node._metadata_bytes += len(chunk)
            buffer.extend(chunk)
            while len(buffer) >= 22:
                record = bytes(buffer[:22])
                del buffer[:22]
                parsed = parse_uvch_record(record)
                if parsed is None:
                    continue
                node._metadata_records += 1
                with node._metadata_lock:
                    node._latest_metadata = parsed
    except Exception as ex:  # pragma: no cover - runtime device errors
        node.get_logger().warn(f"Metadata reader stopped: {ex}")
    finally:
        if node._metadata_proc is not None and node._metadata_proc.poll() is not None:
            node.get_logger().warn(
                f"Metadata stream exited with code {node._metadata_proc.returncode}. "
                "Continuing without new metadata records."
            )


def build_frame_metadata(node, stamp_msg, image_msg) -> dict:
    """Assemble the per-frame metadata payload published alongside images."""
    metadata = {
        "schema_version": 1,
        "source": "mapir_camera_ros2",
        "frame_id": node.frame_id,
        "stamp_sec": int(stamp_msg.sec),
        "stamp_nanosec": int(stamp_msg.nanosec),
        "image_width": int(image_msg.width),
        "image_height": int(image_msg.height),
        "pixel_format": node.pixel_format,
        "video_device": node.video_device,
        "metadata_device": node.metadata_device if node.metadata_enabled else "",
        "published_frame_count": int(node._pub_frames),
        "uvc_controls_device": node._uvc_controls_device_applied,
        "uvc_controls_locked": bool(node._uvc_controls_locked),
        "uvc_controls_requested": dict(node._uvc_controls_requested),
        "uvc_controls_applied": dict(node._uvc_controls_applied),
    }
    if node.metadata_enabled:
        with node._metadata_lock:
            latest = dict(node._latest_metadata)
        metadata.update(latest)
        metadata["metadata_records_seen"] = int(node._metadata_records)
        metadata["metadata_bytes_seen"] = int(node._metadata_bytes)
    return metadata


def publish_metadata(node, payload: dict) -> None:
    """Publish metadata to ROS and optionally append the same payload to JSONL."""
    if node.metadata_pub is None:
        return
    msg = String()
    msg.data = json.dumps(payload, separators=(",", ":"))
    node.metadata_pub.publish(msg)
    if node._metadata_log_file is not None:
        node._metadata_log_file.write(json.dumps(payload) + "\n")
        node._metadata_log_records_since_flush += 1
        if node._metadata_log_records_since_flush >= node.metadata_log_flush_every_n:
            node._metadata_log_file.flush()
            node._metadata_log_records_since_flush = 0


def shutdown_metadata(node) -> None:
    """Stop metadata capture cleanly and flush any pending log output."""
    node._stop_metadata.set()
    if node._metadata_thread is not None:
        node._metadata_thread.join(timeout=0.5)
    if node._metadata_proc is not None and node._metadata_proc.poll() is None:
        node._metadata_proc.terminate()
        try:
            node._metadata_proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            node._metadata_proc.kill()
    if node._metadata_log_file is not None:
        if node._metadata_log_records_since_flush > 0:
            node._metadata_log_file.flush()
        node._metadata_log_file.close()
