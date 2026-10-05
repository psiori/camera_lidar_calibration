#!/usr/bin/env bash
# Build clc_app with Conan-provided dependencies.
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
CONAN_HOME="${CONAN_HOME:-$HOME/.calib-conan}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_REPO="${CALIB_SRC}/camera_lidar_calibration"
BUILD_DIR="${APP_REPO}/build"

export CONAN_HOME

if ! command -v conan >/dev/null 2>&1; then
  echo "Conan not found. Run install-deps.sh first."
  exit 1
fi

if [[ ! -d "${APP_REPO}/.git" ]]; then
  git clone --branch "${CALIB_BRANCH}" \
    "git@github.com:${GITHUB_ORG}/camera_lidar_calibration.git" "${APP_REPO}"
fi

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

cmake --build "${BUILD_DIR}" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"
echo "Built: ${BUILD_DIR}/clc_app/clc_app"
