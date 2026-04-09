FROM ros:jazzy-ros-base

LABEL maintainer="Duda Andrada <duda.andrada@isr.uc.pt>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS Packages
RUN apt update \
    && apt install -y --no-install-recommends \
        git \
        usbutils \
        v4l-utils \
        build-essential \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-tqdm \
        python3-opencv \
        libopencv-dev \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-camera-info-manager-py \
        ros-jazzy-camera-info-manager \
        ros-jazzy-camera-calibration \
        ros-jazzy-tf2-ros \
        ros-jazzy-image-tools \
        ros-jazzy-rviz2 \
        ros-jazzy-rviz-default-plugins \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rmw-cyclonedds-cpp \
        libgl1 \
        libglx-mesa0 \
        libegl1 \
        libxkbcommon-x11-0 \
        mesa-utils \
        x11-apps

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure ROS 2 workspace
ENV ROS2_WS=/workspaces/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# Mount this repo at runtime while debugging (e.g., -v "$PWD":/workspaces/ros2_ws/src/mapir_camera).
RUN mkdir -p ${ROS2_WS}/src/mapir_camera
VOLUME ${ROS2_WS}/src/mapir_camera

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /workspaces/ros2_ws/install/setup.bash" >> /root/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
