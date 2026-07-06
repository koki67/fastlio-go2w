FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    libeigen3-dev \
    libgoogle-glog-dev \
    libomp-dev \
    libpcl-dev \
    libpcap-dev \
    tf2-tools \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd .. && \
    rm -rf build && \
    cd .. && \
    rm -rf Livox-SDK2 && \
    ldconfig
