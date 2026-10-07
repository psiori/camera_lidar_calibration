#!/usr/bin/env bash
# Build clc_app with Conan-provided libraries and Homebrew system dependencies.
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
CONAN_HOME="${CONAN_HOME:-$HOME/.calib-conan}"
CALIB_MARCH_NATIVE="${CALIB_MARCH_NATIVE:-True}"
CMAKE_MARCH_NATIVE=OFF
if [[ "${CALIB_MARCH_NATIVE}" == "True" || "${CALIB_MARCH_NATIVE}" == "ON" || "${CALIB_MARCH_NATIVE}" == "1" ]]; then
  CMAKE_MARCH_NATIVE=ON
fi
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

bash "${SCRIPT_DIR}/ensure-brew-deps.sh"

brew_cmake_prefix_path() {
  local prefix path="" opt
  prefix="$(brew --prefix)"
  for opt in qt@6 qt opencv libomp; do
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
  -o "camera_lidar_calibration/*:clc_dev_build=False"

GENERATORS_DIR="${BUILD_DIR}/build/Release/generators"
BREW_PREFIX_PATH="$(brew_cmake_prefix_path)"
BREW_QT6_DIR="$(brew --prefix qt@6)/lib/cmake/Qt6"
BREW_OPENCV_DIR="$(brew --prefix opencv)/lib/cmake/opencv5"
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
