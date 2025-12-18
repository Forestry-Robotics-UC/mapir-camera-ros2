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
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure colcon workspace
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}/src

# Copy this repo into the workspace
COPY . ${ROS2_WS}/src/mapir_camera

# Build
WORKDIR ${ROS2_WS}
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

# Convenience sourcing
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
