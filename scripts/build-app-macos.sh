#!/usr/bin/env bash
# Build the Qt calibration app shell (clc_app) against an isolated dependency prefix.
set -euo pipefail

CALIB_PREFIX="${CALIB_PREFIX:-$HOME/.local/calib-deps}"
CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
APP_REPO="${CALIB_SRC}/camera_lidar_calibration"
BUILD_DIR="${APP_REPO}/build"

if [[ ! -d "${CALIB_PREFIX}/lib/cmake/gtsam_points" ]]; then
  echo "Missing dependencies in ${CALIB_PREFIX}. Run install-deps-macos.sh first."
  exit 1
fi

export CMAKE_PREFIX_PATH="${CALIB_PREFIX}:$(brew --prefix qt@6):$(brew --prefix):${CMAKE_PREFIX_PATH:-}"
export PATH="$(brew --prefix qt@6)/bin:${PATH}"

if [[ ! -d "${APP_REPO}/.git" ]]; then
  git clone --branch "${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}" \
    "https://github.com/${GITHUB_USER:-themightyoarfish}/camera_lidar_calibration.git" "${APP_REPO}"
fi

cmake -S "${APP_REPO}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
  -Dgtsam_points_DIR="${CALIB_PREFIX}/lib/cmake/gtsam_points" \
  -DCLC_DEV_BUILD=ON \
  -DBUILD_WITH_MARCH_NATIVE=ON

cmake --build "${BUILD_DIR}" -j"$(sysctl -n hw.ncpu)"
echo "Built: ${BUILD_DIR}/clc_app/clc_app"
