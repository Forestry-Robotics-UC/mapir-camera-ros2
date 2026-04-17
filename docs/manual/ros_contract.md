# ROS Contract

## Nodes

- `camera_node` (python entry: `mapir_camera_ros2.camera_node`)
- `indices_node` (python entry: `mapir_camera_ros2.indices_node`)

## Topics

All topics are relative to the namespace (launch default: `namespace:=mapir`).

| Topic | Type | Notes |
|---|---|---|
| `/<ns>/image_raw` | `sensor_msgs/Image` | Encoding `bgr8`. |
| `/<ns>/camera_info` | `sensor_msgs/CameraInfo` | Loaded via `camera_info_manager` if provided. |
| `/<ns>/metadata` | `std_msgs/String` | JSON payload; enabled with `metadata_enabled=true`. |
| `/<ns>/indices/<index>` | `sensor_msgs/Image` | Encoding `32FC1` (float32). |
| `/<ns>/indices_color/<index>` | `sensor_msgs/Image` | Optional; encoding `bgr8` (colormap). |

## QoS

- Default: BEST_EFFORT, depth 5 (camera-stream friendly; RViz compatible).

## Frames

- `frame_id` defaults to `mapir3_optical_frame` (REP-105 optical frame convention).

## Timestamps

- Frames are stamped using the node clock at publish time (not a hardware timestamp).
- Metadata topic includes ROS timestamp plus best-effort UVC timing fields from
  `metadata_device` (e.g., `uvc_pts`, `uvc_scr_stc`) when available.
- When startup UVC lock is enabled, metadata also includes:
  - `uvc_controls_device`
  - `uvc_controls_locked`
  - `uvc_controls_requested`
  - `uvc_controls_applied`

## Runtime Toggles

- `indices_node` supports a runtime `enabled` parameter:
  - `ros2 param set /<ns>/indices enabled false`
  - `ros2 param set /<ns>/indices enabled true`
