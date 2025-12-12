# MAPIR Survey3 ROS 2 Camera Driver (Jazzy)

ROS 2 Jazzy camera driver for MAPIR Survey3 cameras (including OCN variants),
implemented in Python (ament-python) using OpenCV + V4L2.

This package publishes synchronized camera streams suitable for real-time robotics
pipelines and vegetation analysis workflows.

---

## Features

- ROS 2 Jazzy compatible
- Publishes:
  - /<ns>/image_raw (sensor_msgs/Image)
  - /<ns>/camera_info (sensor_msgs/CameraInfo)
- Supports MJPG and H264 pixel formats (device-dependent)
- Uses V4L2 backend explicitly (no GStreamer ambiguity)
- Supports up to 60 Hz
- BEST_EFFORT QoS by default (RViz / rqt compatible)
- Uses camera_info_manager
  - Publishes default CameraInfo if calibration is missing
- Extensive debug logging (rate, failures, timing)
- Designed for Jetson / embedded and desktop Linux

---

## Topics

All topics are published relative to the node namespace (recommended: /mapir).

- /mapir/image_raw
- /mapir/camera_info

---

## Parameters

debug (bool, default false)  
Enable verbose debug logging.

debug_period_s (double, default 1.0)  
Throttle period for debug logs.

video_device (string, default /dev/video0)  
Camera device path or index.

image_width (int, default 1920)  
Requested image width.

image_height (int, default 1440)  
Requested image height.

framerate (double, default 30.0)  
Target frame rate.

pixel_format (string, default MJPG)  
MJPG or H264.

frame_id (string, default mapir3_optical_frame)  
Optical frame ID.

camera_name (string, default mapir3)  
Camera name for calibration.

camera_info_url (string, default empty)  
file:///.../calibration.yaml

qos_best_effort (bool, default true)  
BEST_EFFORT vs RELIABLE QoS.

qos_depth (int, default 5)  
Publisher queue depth.

---

## Calibration

Calibration files follow the standard ROS camera calibration format.

Example path:
~/.ros/camera_info/mapir3.yaml

Run with:
-p camera_info_url:=file:///root/.ros/camera_info/mapir3.yaml

If calibration is missing or invalid, the node continues publishing
with a valid but uncalibrated CameraInfo.

---

## Running the Node

Direct execution:

ros2 run mapir_camera_ros2 camera_node --ros-args -r __ns:=/mapir -p debug:=true

Debug with explicit log level:

ros2 run mapir_camera_ros2 camera_node --ros-args --log-level mapir_camera:=debug -r __ns:=/mapir -p debug:=true

---

## Launch File

ros2 launch mapir_camera_ros2 mapir_camera.launch.py namespace:=mapir debug:=true qos_best_effort:=true camera_info_url:=file:///root/.ros/camera_info/mapir3.yaml

---

## Viewing the Image Stream

Recommended (headless / container-safe):

ros2 run rqt_image_view rqt_image_view

Select:
/mapir/image_raw

---

## Debugging Tips

Check negotiated formats:

v4l2-ctl --device=/dev/video0 --list-formats-ext

Verify publishing:

ros2 topic info /mapir/image_raw -v  
ros2 topic hz /mapir/image_raw

Run without rebuild (fast iteration):

python3 -m mapir_camera_ros2.camera_node --ros-args -r __ns:=/mapir -p debug:=true

---

## License

MIT License

© Duda Andrada

---

## Maintainer

Duda Andrada  
duda.andrada@isr.uc.pt
