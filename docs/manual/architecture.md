# Architecture

![Architecture placeholder](../_static/architecture_placeholder.svg)

## Repository layout

- `mapir_camera_ros2/`: ROS-facing Python nodes (rclpy publishers/subscribers, parameters, QoS).
- `mapir_camera_cpp/`: ROS-facing C++ node (rclcpp publishers/subscribers).
- `mapir_camera_core/`: ROS-agnostic logic:
  - V4L2/OpenCV capture helpers (`v4l2_camera.py`)
  - multispectral index math (`spectral_indices.py`)

## Data flow

1. `camera_node` captures frames from `/dev/video*` and publishes:
   - `/<ns>/image_raw` (`sensor_msgs/Image`)
   - `/<ns>/camera_info` (`sensor_msgs/CameraInfo`)
2. Optional `indices_node` subscribes to `/<ns>/image_raw` and publishes:
   - `/<ns>/indices/<index>` (`sensor_msgs/Image`, `32FC1`)

## Design intent

Keep ROS-independent processing in `mapir_camera_core` so it can be reused later
for non-ROS interfaces (e.g., HTTP SDK, file batch processing).
