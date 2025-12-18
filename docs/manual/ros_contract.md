# ROS Contract

![ROS graph placeholder](../_static/ros_graph_placeholder.svg)

## Nodes

- `camera_node` (python entry: `mapir_camera_ros2.camera_node`)
- `indices_node` (python entry: `mapir_camera_ros2.indices_node`)

## Topics

All topics are relative to the namespace (launch default: `namespace:=mapir`).

| Topic | Type | Notes |
|---|---|---|
| `/<ns>/image_raw` | `sensor_msgs/Image` | Encoding `bgr8`. |
| `/<ns>/camera_info` | `sensor_msgs/CameraInfo` | Loaded via `camera_info_manager` if provided. |
| `/<ns>/indices/<index>` | `sensor_msgs/Image` | Encoding `32FC1` (float32). |
| `/<ns>/indices_color/<index>` | `sensor_msgs/Image` | Optional; encoding `bgr8` (colormap). |

## QoS

- Default: BEST_EFFORT, depth 5 (camera-stream friendly; RViz compatible).

## Frames

- `frame_id` defaults to `mapir3_optical_frame` (REP-105 optical frame convention).

## Timestamps

- Frames are stamped using the node clock at publish time (not a hardware timestamp).

## Runtime Toggles

- `indices_node` supports a runtime `enabled` parameter:
  - `ros2 param set /<ns>/indices enabled false`
  - `ros2 param set /<ns>/indices enabled true`
