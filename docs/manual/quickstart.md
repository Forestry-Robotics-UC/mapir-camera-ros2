# Quickstart

![RViz placeholder](../_static/rviz_placeholder.svg)

## 1) Verify the camera is visible

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
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
  image_width:=1920 image_height:=1440 \
  framerate:=30.0 \
  pixel_format:=MJPG
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

## 5) Enable indices (Survey3W OCN)

```bash
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  enable_indices:=true \
  indices_params_file:=$(ros2 pkg prefix mapir_camera_ros2)/share/mapir_camera_ros2/config/mapir_indices_ocn_params.yaml
```

## 6) Visualize

- RViz: add an Image display and select `/mapir/image_raw`.
- Quick viewer:
  - `ros2 run image_tools showimage --ros-args -r image:=/mapir/image_raw`

