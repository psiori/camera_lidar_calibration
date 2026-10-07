#!/usr/bin/env bash
# Build and cache calibration library dependencies with Conan + Homebrew system libs.
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
CONAN_HOME="${CONAN_HOME:-$HOME/.calib-conan}"
CALIB_MARCH_NATIVE="${CALIB_MARCH_NATIVE:-True}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export CONAN_HOME

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

macos_toolchain_conf_args() {
  MACOS_TOOLCHAIN_CONF=()
  if [[ "$(uname -s)" != "Darwin" ]]; then
    return
  fi
  while IFS= read -r -d '' arg; do
    MACOS_TOOLCHAIN_CONF+=("${arg}")
  done < <(bash "${SCRIPT_DIR}/conan-macos-toolchain-args.sh" || true)
}

clone_or_update() {
  bash "${SCRIPT_DIR}/clone-repos.sh"
}

ensure_conan
bash "${SCRIPT_DIR}/ensure-brew-deps.sh"
conan profile detect --force >/dev/null 2>&1 || true

mkdir -p "${CALIB_SRC}" "${CONAN_HOME}"

clone_or_update

read -r -a PROFILE <<< "$(profile_args)"
macos_toolchain_conf_args
BUILD_PROFILE=(-pr:b=default)
CONAN_BUILD=(--build=missing)

echo "Using CONAN_HOME=${CONAN_HOME}"
echo "Building Conan packages from ${CALIB_SRC}"
echo "Stack toolchain (OpenCV, OpenMP, Qt hints) comes from camera_lidar_calibration/conan/."

conan create "${CALIB_SRC}/gtsam" --name=gtsam --version=4.3a1 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${MACOS_TOOLCHAIN_CONF[@]}" "${CONAN_BUILD[@]}" \
  -o "gtsam/*:build_with_march_native=${CALIB_MARCH_NATIVE}"

conan create "${CALIB_SRC}/gtsam_points" --name=gtsam_points --version=1.2.2 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${MACOS_TOOLCHAIN_CONF[@]}" "${CONAN_BUILD[@]}" \
  -o "gtsam_points/*:build_with_march_native=${CALIB_MARCH_NATIVE}" \
  -o "gtsam_points/*:build_with_cuda=False"

conan create "${CALIB_SRC}/glim" --name=glim --version=1.2.2 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${MACOS_TOOLCHAIN_CONF[@]}" "${CONAN_BUILD[@]}" \
  -o "glim/*:build_with_viewer=False" \
  -o "glim/*:build_with_cuda=False" \
  -o "glim/*:build_with_march_native=${CALIB_MARCH_NATIVE}" \
  -o "glim/*:build_glim_cloud_fusion=True"

conan create "${CALIB_SRC}/direct_visual_lidar_calibration" --name=vlcal_align --version=0.1.0 \
  -s build_type=Release "${BUILD_PROFILE[@]}" "${PROFILE[@]}" "${MACOS_TOOLCHAIN_CONF[@]}" "${CONAN_BUILD[@]}" \
  -o "vlcal_align/*:build_with_viewer=False" \
  -o "vlcal_align/*:build_with_march_native=${CALIB_MARCH_NATIVE}" \
  -o "vlcal_align/*:build_vlcal_preprocess=True"

echo "Conan dependency packages are ready in ${CONAN_HOME}"
echo "Next: CALIB_SRC=${CALIB_SRC} bash ${CLC_ROOT}/scripts/build-app.sh"
