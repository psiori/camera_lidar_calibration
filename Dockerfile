FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git python3 python3-pip python3-venv \
    libeigen3-dev libboost-all-dev libtbb-dev libomp-dev \
    qt6-base-dev qt6-declarative-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages conan==2.* \
    && conan profile detect --force

WORKDIR /workspace
COPY direct_visual_lidar_calibration /workspace/direct_visual_lidar_calibration
COPY glim /workspace/glim
COPY gtsam_points /workspace/gtsam_points
COPY gtsam /workspace/gtsam
COPY camera_lidar_calibration /workspace/camera_lidar_calibration

ENV CONAN_HOME=/opt/conan-prefix
RUN bash /workspace/camera_lidar_calibration/scripts/conan-prefix.sh
