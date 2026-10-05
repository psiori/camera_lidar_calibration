#!/usr/bin/env bash
# Build and cache all calibration library dependencies with Conan.
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
CONAN_HOME="${CONAN_HOME:-$HOME/.calib-conan}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export CONAN_HOME

# Prevent Homebrew/system CMake packages from shadowing Conan dependencies.
unset CMAKE_PREFIX_PATH
unset PKG_CONFIG_PATH

ensure_conan() {
  if command -v conan >/dev/null 2>&1; then
    return
  fi
  if command -v pip3 >/dev/null 2>&1; then
    pip3 install --user 'conan>=2,<3'
    export PATH="${HOME}/.local/bin:${PATH}"
  elif command -v pipx >/dev/null 2>&1; then
    pipx install 'conan>=2,<3'
  else
    echo "Conan 2 is required. Install with: pip3 install 'conan>=2,<3'"
    exit 1
  fi
}

profile_args() {
  if [[ "$(uname -s)" == "Darwin" ]] && [[ -f "${CLC_ROOT}/conan/profiles/native-macos" ]]; then
    echo "-pr:h=${CLC_ROOT}/conan/profiles/native-macos"
  elif [[ -f "${CLC_ROOT}/conan/profiles/native" ]]; then
    echo "-pr:h=${CLC_ROOT}/conan/profiles/native"
  else
    echo "-pr:h=default"
  fi
}

clone_or_update() {
  local repo="$1"
  local url="git@github.com:${GITHUB_ORG}/${repo}.git"
  if [[ -d "${CALIB_SRC}/${repo}/.git" ]]; then
    git -C "${CALIB_SRC}/${repo}" fetch origin
    git -C "${CALIB_SRC}/${repo}" checkout "${CALIB_BRANCH}" 2>/dev/null \
      || git -C "${CALIB_SRC}/${repo}" checkout -B "${CALIB_BRANCH}" "origin/${CALIB_BRANCH}"
    git -C "${CALIB_SRC}/${repo}" pull --ff-only origin "${CALIB_BRANCH}" || true
  else
    git clone --branch "${CALIB_BRANCH}" "${url}" "${CALIB_SRC}/${repo}"
  fi
}

ensure_conan
conan profile detect --force >/dev/null 2>&1 || true

mkdir -p "${CALIB_SRC}" "${CONAN_HOME}"

for repo in gtsam gtsam_points glim direct_visual_lidar_calibration; do
  clone_or_update "${repo}"
done

read -r -a PROFILE <<< "$(profile_args)"
BUILD_PROFILE=(-pr:b=default)
MARCH_OPTS=(-o "&:build_with_march_native=True" -o "gtsam/*:build_with_march_native=True")
OPENCV_OPTS=(-o "opencv/*:with_ffmpeg=False" -o "opencv/*:with_gtk=False")
SPDLOG_OPTS=(-o "spdlog/*:header_only=False")
CONAN_BUILD=(--build=missing)

echo "Using CONAN_HOME=${CONAN_HOME}"
echo "Building Conan packages from ${CALIB_SRC}"

conan create "${CALIB_SRC}/gtsam" --name=gtsam --version=4.3a1 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${MARCH_OPTS[@]}" "${CONAN_BUILD[@]}"

conan create "${CALIB_SRC}/gtsam_points" --name=gtsam_points --version=1.2.2 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${CONAN_BUILD[@]}" \
  -o "gtsam_points/*:build_with_march_native=True" \
  -o "gtsam_points/*:build_with_cuda=False"

conan create "${CALIB_SRC}/glim" --name=glim --version=1.2.2 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${CONAN_BUILD[@]}" "${OPENCV_OPTS[@]}" "${SPDLOG_OPTS[@]}" \
  -o "glim/*:build_with_viewer=False" \
  -o "glim/*:build_with_cuda=False" \
  -o "glim/*:build_with_march_native=True" \
  -o "glim/*:build_glim_cloud_fusion=True"

conan create "${CALIB_SRC}/direct_visual_lidar_calibration" --name=vlcal_align --version=0.1.0 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${CONAN_BUILD[@]}" "${OPENCV_OPTS[@]}" \
  -o "vlcal_align/*:build_with_viewer=False" \
  -o "vlcal_align/*:build_with_march_native=True" \
  -o "vlcal_align/*:build_vlcal_preprocess=True"

echo "Conan dependency packages are ready in ${CONAN_HOME}"
echo "Next: CALIB_SRC=${CALIB_SRC} bash ${CLC_ROOT}/scripts/build-app.sh"
