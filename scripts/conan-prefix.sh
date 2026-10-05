#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONAN_HOME="${CONAN_HOME:-${ROOT}/camera_lidar_calibration/.conan-prefix}"
export CONAN_HOME

PROFILE_ARGS=(-pr:b=default -pr:h=default)
if [[ "$(uname -s)" == "Darwin" ]] && [[ -f "${ROOT}/camera_lidar_calibration/conan/profiles/native-macos" ]]; then
  PROFILE_ARGS=(-pr:h="${ROOT}/camera_lidar_calibration/conan/profiles/native-macos")
elif [[ -f "${ROOT}/camera_lidar_calibration/conan/profiles/native" ]]; then
  PROFILE_ARGS=(-pr:h="${ROOT}/camera_lidar_calibration/conan/profiles/native")
fi

echo "Using CONAN_HOME=${CONAN_HOME}"

GTSAM_RECIPE="${ROOT}/gtsam"
if [[ ! -f "${GTSAM_RECIPE}/conanfile.py" ]]; then
  GTSAM_RECIPE="${ROOT}/camera_lidar_calibration/conan/gtsam"
fi
conan create "${GTSAM_RECIPE}" --name=gtsam --version=4.3a1 -s build_type=Release "${PROFILE_ARGS[@]}" \
  -o "&:build_with_march_native=True" \
  -o gtsam/*:build_with_march_native=True

conan create "${ROOT}/gtsam_points" --name=gtsam_points --version=1.2.2 -s build_type=Release "${PROFILE_ARGS[@]}" \
  -o "&:build_with_march_native=True" \
  -o gtsam_points/*:build_with_march_native=True

conan create "${ROOT}/glim" --name=glim --version=1.2.2 -s build_type=Release "${PROFILE_ARGS[@]}" \
  -o glim/*:build_with_viewer=False \
  -o glim/*:build_with_cuda=False \
  -o glim/*:build_with_march_native=True \
  -o glim/*:build_glim_cloud_fusion=True

conan create "${ROOT}/direct_visual_lidar_calibration" --name=vlcal_align --version=0.1.0 -s build_type=Release "${PROFILE_ARGS[@]}" \
  -o vlcal_align/*:build_with_viewer=False \
  -o vlcal_align/*:build_with_march_native=True \
  -o vlcal_align/*:build_vlcal_preprocess=True

conan create "${ROOT}/camera_lidar_calibration" --name=camera_lidar_calibration --version=0.1.0 -s build_type=Release "${PROFILE_ARGS[@]}" \
  -o camera_lidar_calibration/*:build_with_march_native=True

BUILD_DIR="${ROOT}/camera_lidar_calibration/build"
cmake -S "${ROOT}/camera_lidar_calibration" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${CONAN_HOME}"
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"

echo "Build and tests completed."
