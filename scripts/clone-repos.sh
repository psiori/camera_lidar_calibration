#!/usr/bin/env bash
# Clone or update all calibration repositories under CALIB_SRC.
set -euo pipefail

CALIB_SRC="${CALIB_SRC:-$HOME/source_builds}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
GIT_URL_SCHEME="${GIT_URL_SCHEME:-ssh}"

repos=(
  gtsam
  gtsam_points
  glim
  direct_visual_lidar_calibration
  camera_lidar_calibration
)

repo_url() {
  local repo="$1"
  if [[ "${GIT_URL_SCHEME}" == "https" ]]; then
    echo "https://github.com/${GITHUB_ORG}/${repo}.git"
  else
    echo "git@github.com:${GITHUB_ORG}/${repo}.git"
  fi
}

init_submodules() {
  local repo="$1"
  if [[ -f "${CALIB_SRC}/${repo}/.gitmodules" ]]; then
    echo "Initializing submodules for ${repo}..."
    git -C "${CALIB_SRC}/${repo}" submodule update --init --recursive
  fi
}

clone_or_update() {
  local repo="$1"
  local url
  url="$(repo_url "${repo}")"
  if [[ -d "${CALIB_SRC}/${repo}/.git" ]]; then
    echo "Updating ${repo}..."
    git -C "${CALIB_SRC}/${repo}" fetch origin
    git -C "${CALIB_SRC}/${repo}" checkout "${CALIB_BRANCH}" 2>/dev/null \
      || git -C "${CALIB_SRC}/${repo}" checkout -B "${CALIB_BRANCH}" "origin/${CALIB_BRANCH}"
    git -C "${CALIB_SRC}/${repo}" pull --ff-only origin "${CALIB_BRANCH}" || true
  else
    echo "Cloning ${repo}..."
    git clone --recurse-submodules --branch "${CALIB_BRANCH}" "${url}" "${CALIB_SRC}/${repo}"
  fi
  init_submodules "${repo}"
}

mkdir -p "${CALIB_SRC}"

echo "CALIB_SRC=${CALIB_SRC}"
echo "CALIB_BRANCH=${CALIB_BRANCH}"
echo "GITHUB_ORG=${GITHUB_ORG}"

for repo in "${repos[@]}"; do
  clone_or_update "${repo}"
done

echo "Repositories ready under ${CALIB_SRC}"
