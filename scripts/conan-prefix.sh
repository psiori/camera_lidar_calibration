#!/usr/bin/env bash
#
# conan-prefix.sh — Full build-and-test using a project-local Conan cache.
#
# What this script does, step by step:
#   1. Set CALIB_SRC to the parent of this repo (or use the CALIB_SRC env var).
#   2. Point CONAN_HOME at <CALIB_SRC>/camera_lidar_calibration/.conan-prefix
#      so packages are cached inside the project tree instead of ~/.calib-conan.
#   3. Run install-deps.sh to build all library dependencies into that local cache.
#   4. Run `conan install` for the camera_lidar_calibration app with clc_app enabled.
#   5. Configure CMake with the generated Conan toolchain (Ninja, Release).
#   6. Build the project.
#   7. Run ctest to verify the build.
#
# Use this for isolated, reproducible builds where the Conan cache lives in-repo.
# For the default shared cache workflow, use install-deps.sh + build-app.sh instead.
#
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
export CALIB_SRC
export CONAN_HOME="${CONAN_HOME:-${CALIB_SRC}/camera_lidar_calibration/.conan-prefix}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "${SCRIPT_DIR}/install-deps.sh"

APP_REPO="${CALIB_SRC}/camera_lidar_calibration"
BUILD_DIR="${APP_REPO}/build"
PROFILE_ARGS=()
if [[ "$(uname -s)" == "Darwin" ]] && [[ -f "${APP_REPO}/conan/profiles/native-macos" ]]; then
  PROFILE_ARGS=(-pr:h="${APP_REPO}/conan/profiles/native-macos")
elif [[ -f "${APP_REPO}/conan/profiles/native" ]]; then
  PROFILE_ARGS=(-pr:h="${APP_REPO}/conan/profiles/native")
fi

conan install "${APP_REPO}" \
  --build=missing \
  -s build_type=Release \
  "${PROFILE_ARGS[@]}" \
  -of "${BUILD_DIR}" \
  -o "camera_lidar_calibration/*:build_clc_app=True" \
  -o "camera_lidar_calibration/*:build_with_march_native=True" \
  -o "camera_lidar_calibration/*:clc_dev_build=False"

cmake -S "${APP_REPO}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="${BUILD_DIR}/conan_toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="${BUILD_DIR}" \
  -DCLC_DEV_BUILD=OFF

cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"

echo "Build and tests completed."
