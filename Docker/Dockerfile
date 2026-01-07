FROM ros:jazzy-ros-base

LABEL maintainer="Duda Andrada <duda.andrada@isr.uc.pt> "

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS + system dependencies for mapir_camera_ros2
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        usbutils \
        v4l-utils \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-camera-info-manager-py \
        ros-jazzy-tf2-ros \
        ros-jazzy-image-tools \
        ros-jazzy-rviz2 \
        ros-jazzy-rviz-default-plugins \
        ros-jazzy-rmw-cyclonedds-cpp \
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        libxkbcommon-x11-0 \
        mesa-utils \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure colcon workspace
ENV ROS2_WS=/workspaces/ros2_ws
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}/src

# Mount this repo at runtime while debugging (e.g., -v "$PWD":/workspaces/ros2_ws/src/mapir_camera).
RUN mkdir -p ${ROS2_WS}/src/mapir_camera
VOLUME ${ROS2_WS}/src/mapir_camera

# Build
WORKDIR ${ROS2_WS}
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

# Convenience sourcing
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /workspaces/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
