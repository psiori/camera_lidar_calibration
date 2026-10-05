FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages 'conan>=2,<3' \
    && conan profile detect --force

WORKDIR /workspace
COPY gtsam /workspace/gtsam
COPY gtsam_points /workspace/gtsam_points
COPY glim /workspace/glim
COPY direct_visual_lidar_calibration /workspace/direct_visual_lidar_calibration
COPY camera_lidar_calibration /workspace/camera_lidar_calibration

ENV CALIB_SRC=/workspace
ENV CONAN_HOME=/opt/conan-prefix
RUN bash /workspace/camera_lidar_calibration/scripts/conan-prefix.sh
