#!/usr/bin/env bash
# Install calibration libraries into an isolated prefix (default: ~/.local/calib-deps).
# Does not install the Qt app — run build-app-macos.sh after this.
set -euo pipefail

CALIB_PREFIX="${CALIB_PREFIX:-$HOME/.local/calib-deps}"
CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_USER="${GITHUB_USER:-themightyoarfish}"

echo "CALIB_PREFIX=${CALIB_PREFIX}"
echo "CALIB_SRC=${CALIB_SRC}"
mkdir -p "${CALIB_PREFIX}" "${CALIB_SRC}"

if ! command -v brew >/dev/null; then
  echo "Homebrew is required: https://brew.sh"
  exit 1
fi

brew install cmake ninja eigen boost fmt spdlog opencv pcl nlohmann-json qt@6 2>/dev/null || true
brew link --force fmt spdlog qt@6 2>/dev/null || true

export CMAKE_PREFIX_PATH="${CALIB_PREFIX}:$(brew --prefix):${CMAKE_PREFIX_PATH:-}"
export PATH="$(brew --prefix qt@6)/bin:${PATH}"

clone_or_update() {
  local dir="$1"
  local url="https://github.com/${GITHUB_USER}/${dir}.git"
  if [[ -d "${CALIB_SRC}/${dir}/.git" ]]; then
    git -C "${CALIB_SRC}/${dir}" fetch origin
    git -C "${CALIB_SRC}/${dir}" checkout "${BRANCH}" 2>/dev/null || git -C "${CALIB_SRC}/${dir}" checkout -B "${BRANCH}" "origin/${BRANCH}"
    git -C "${CALIB_SRC}/${dir}" pull --ff-only origin "${BRANCH}" || true
  else
    git clone --branch "${BRANCH}" "${url}" "${CALIB_SRC}/${dir}"
  fi
}

# GTSAM (upstream tag; no fork required)
if [[ ! -d "${CALIB_SRC}/gtsam/.git" ]]; then
  git clone --depth 1 --branch 4.3a1 https://github.com/borglab/gtsam.git "${CALIB_SRC}/gtsam"
fi
cmake -S "${CALIB_SRC}/gtsam" -B "${CALIB_SRC}/gtsam/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CALIB_PREFIX}" \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON
cmake --build "${CALIB_SRC}/gtsam/build" -j"$(sysctl -n hw.ncpu)"
cmake --install "${CALIB_SRC}/gtsam/build"

clone_or_update "gtsam_points"
cmake -S "${CALIB_SRC}/gtsam_points" -B "${CALIB_SRC}/gtsam_points/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CALIB_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${CALIB_PREFIX}:$(brew --prefix)" \
  -DBUILD_WITH_MARCH_NATIVE=ON
cmake --build "${CALIB_SRC}/gtsam_points/build" -j"$(sysctl -n hw.ncpu)"
cmake --install "${CALIB_SRC}/gtsam_points/build"

clone_or_update "glim"
cmake -S "${CALIB_SRC}/glim" -B "${CALIB_SRC}/glim/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CALIB_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${CALIB_PREFIX}:$(brew --prefix)" \
  -Dgtsam_points_DIR="${CALIB_PREFIX}/lib/cmake/gtsam_points" \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_MARCH_NATIVE=ON \
  -DBUILD_GLIM_CLOUD_FUSION=ON \
  -DBUILD_GLIM_CLOUD_FUSION_TESTS=OFF
cmake --build "${CALIB_SRC}/glim/build" -j"$(sysctl -n hw.ncpu)"
cmake --install "${CALIB_SRC}/glim/build"

clone_or_update "direct_visual_lidar_calibration"
cmake -S "${CALIB_SRC}/direct_visual_lidar_calibration" -B "${CALIB_SRC}/direct_visual_lidar_calibration/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CALIB_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${CALIB_PREFIX}:$(brew --prefix)" \
  -DBUILD_VLCAL_PREPROCESS=ON \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_MARCH_NATIVE=ON \
  -DBUILD_VLCAL_TESTS=OFF
cmake --build "${CALIB_SRC}/direct_visual_lidar_calibration/build" -j"$(sysctl -n hw.ncpu)"
cmake --install "${CALIB_SRC}/direct_visual_lidar_calibration/build"

echo "Dependencies installed to ${CALIB_PREFIX}"
echo "Next: CALIB_PREFIX=${CALIB_PREFIX} ${CALIB_SRC}/camera_lidar_calibration/scripts/build-app-macos.sh"
