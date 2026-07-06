FROM arm64v8/ubuntu:jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release git software-properties-common

RUN rm /etc/apt/apt.conf.d/docker-clean

ARG ROS_DISTRO=humble
ARG INSTALL_PACKAGE=desktop

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
      python3-argcomplete \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-numpy

RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      iputils-ping \
      git \
      tmux \
      vim \
      nano \
      wget \
      python3-pip \
      python3-dev \
      libboost-all-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libomp-dev \
      libpcl-dev \
      libpcap-dev \
      ros-humble-pcl-conversions \
      ros-humble-pcl-ros \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-default-generators \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-ros \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs

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
RUN python3 -m pip install --no-cache-dir catmux

RUN rm -rf /var/lib/apt/lists/*
