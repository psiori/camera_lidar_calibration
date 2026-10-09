#!/usr/bin/env bash
#
# build-app.sh — Configure and build the camera_lidar_calibration application (clc_app).
#
# What this script does, step by step:
#   1. Resolve paths and environment variables (CALIB_SRC, CONAN_HOME, build flags).
#   2. Verify Conan is installed (run install-deps.sh first if not).
#   3. Clone the camera_lidar_calibration app repo if it is not already present.
#   4. Install macOS Homebrew packages via ensure-brew-deps.sh.
#   5. Select the Conan host profile (native-macos on Darwin, native elsewhere).
#   6. On macOS, load OpenCV/OpenMP/Qt toolchain hints from conan-macos-toolchain-args.sh.
#   7. Verify the Conan cache has the required packages (gtsam, etc.).
#   8. Run `conan install` to generate CMake toolchain files and fetch dependencies.
#   9. Configure CMake with Ninja, pointing at the Conan toolchain and Homebrew Qt/OpenCV.
#  10. Build clc_app with all available CPU cores.
#  11. Print the path to the built binary.
#
# Prerequisites: run install-deps.sh first to populate the Conan cache.
#
# Environment variables (all optional):
#   CALIB_SRC          — Parent directory holding all cloned repos.
#   CONAN_HOME         — Conan cache directory (default: ~/.calib-conan).
#   CALIB_BRANCH       — Git branch for the app repo.
#   GITHUB_ORG         — GitHub org for cloning.
#   CALIB_MARCH_NATIVE — Enable -march=native in the app build (default: True).
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_SRC="${CALIB_SRC:-$(cd "${CLC_ROOT}/.." && pwd)}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
CONAN_HOME="${CONAN_HOME:-$HOME/.calib-conan}"
CALIB_MARCH_NATIVE="${CALIB_MARCH_NATIVE:-True}"
CMAKE_MARCH_NATIVE=OFF
if [[ "${CALIB_MARCH_NATIVE}" == "True" || "${CALIB_MARCH_NATIVE}" == "ON" || "${CALIB_MARCH_NATIVE}" == "1" ]]; then
  CMAKE_MARCH_NATIVE=ON
fi
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

bash "${SCRIPT_DIR}/ensure-brew-deps.sh"

brew_cmake_prefix_path() {
  local prefix path="" opt
  prefix="$(brew --prefix)"
  for opt in qt@6 qt opencv@4 libomp; do
    if [[ -d "${prefix}/opt/${opt}" ]]; then
      if [[ -n "${path}" ]]; then
        path="${path};${prefix}/opt/${opt}"
      else
        path="${prefix}/opt/${opt}"
      fi
    fi
  done
  echo "${path}"
}

PROFILE_ARGS=(-pr:b=default)
if [[ "$(uname -s)" == "Darwin" ]] && [[ -f "${APP_REPO}/conan/profiles/native-macos" ]]; then
  PROFILE_ARGS+=(-pr:h="${APP_REPO}/conan/profiles/native-macos")
elif [[ -f "${APP_REPO}/conan/profiles/native" ]]; then
  PROFILE_ARGS+=(-pr:h="${APP_REPO}/conan/profiles/native")
fi

MACOS_TOOLCHAIN_CONF=()
if [[ "$(uname -s)" == "Darwin" ]]; then
  while IFS= read -r -d '' arg; do
    MACOS_TOOLCHAIN_CONF+=("${arg}")
  done < <(bash "${SCRIPT_DIR}/conan-macos-toolchain-args.sh" || true)
fi

if [[ ! -d "${CONAN_HOME}/p" ]] && ! conan list "gtsam/4.3a1" 2>/dev/null | grep -q gtsam; then
  echo "Conan cache is empty. Run install-deps.sh first."
  exit 1
fi

conan install "${APP_REPO}" \
  --build=missing \
  -s build_type=Release \
  "${PROFILE_ARGS[@]}" \
  "${MACOS_TOOLCHAIN_CONF[@]}" \
  -of "${BUILD_DIR}" \
  -o "camera_lidar_calibration/*:build_clc_app=True" \
  -o "camera_lidar_calibration/*:build_with_march_native=${CALIB_MARCH_NATIVE}" \
  -o "camera_lidar_calibration/*:clc_dev_build=False" \
  -o "vlcal_align/*:shared=False" \
  -o "vlcal_align/*:build_with_viewer=False" \
  -o "vlcal_align/*:build_with_march_native=${CALIB_MARCH_NATIVE}" \
  -o "vlcal_align/*:build_vlcal_preprocess=True"

GENERATORS_DIR="${BUILD_DIR}/build/Release/generators"
BREW_PREFIX_PATH="$(brew_cmake_prefix_path)"
BREW_QT6_DIR="$(brew --prefix qt@6)/lib/cmake/Qt6"
BREW_OPENCV_DIR="$(brew --prefix opencv@4)/lib/cmake/opencv4"
CMAKE_EXTRA_ARGS=()
if [[ -d "${BREW_QT6_DIR}" ]]; then
  CMAKE_EXTRA_ARGS+=(-DQt6_DIR="${BREW_QT6_DIR}")
fi
if [[ -d "${BREW_OPENCV_DIR}" ]]; then
  CMAKE_EXTRA_ARGS+=(-DOpenCV_DIR="${BREW_OPENCV_DIR}")
fi
cmake -S "${APP_REPO}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="${GENERATORS_DIR}/conan_toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="${BREW_PREFIX_PATH};${GENERATORS_DIR}" \
  -DCLC_DEV_BUILD=OFF \
  -DBUILD_WITH_MARCH_NATIVE="${CMAKE_MARCH_NATIVE}" \
  "${CMAKE_EXTRA_ARGS[@]}"

cmake --build "${BUILD_DIR}" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"
echo "Built: ${BUILD_DIR}/clc_app/clc_app"
