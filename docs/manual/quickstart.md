# Quickstart


## 1) Verify the camera is visible

```bash
v4l2-ctl --list-formats-ext -d /dev/mapir
```

## 2) Build from source (colcon)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3) Run the camera node (baseline 30 Hz)

```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  video_device:=/dev/video0 \
  image_width:=1280 image_height:=720 \
  framerate:=30.0 \
  pixel_format:=MJPG
```

GStreamer capture (optional):
```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  use_gstreamer:=true \
  gstreamer_pipeline:="v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
```

Validate:

```bash
ros2 topic hz /mapir/image_raw
ros2 topic echo --once /mapir/image_raw --field header
```

## 4) Enable indices (RGN baseline)

```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  enable_indices:=true
```
For real-time performance, keep the indices list to 1-2 entries.

## 5) Enable indices (Survey3W OCN)

```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  enable_indices:=true \
  indices_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_indices_ocn_params.yaml
```

## 6) Visualize

- RViz: add an Image display and select `/mapir/image_raw`.
  - If the image does not show, set the Image display QoS Reliability to
    `Best Effort` (matches the default publisher QoS).
  - Alternatively launch with `qos_best_effort:=false` to publish RELIABLE.
- Quick viewer:
  - `ros2 run image_tools showimage --ros-args -r image:=/mapir/image_raw`
