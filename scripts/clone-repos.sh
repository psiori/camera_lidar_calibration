#!/usr/bin/env bash
#
# clone-repos.sh — Clone or update all calibration source repositories.
#
# What this script does, step by step:
#   1. Resolve CALIB_SRC (parent directory for all repos) and git settings.
#   2. Create CALIB_SRC if it does not exist.
#   3. For each repo in the stack (gtsam, gtsam_points, glim,
#      direct_visual_lidar_calibration, camera_lidar_calibration):
#        a. If already cloned: fetch origin, check out CALIB_BRANCH, optionally pull.
#        b. If not cloned: git clone with --recurse-submodules on CALIB_BRANCH.
#        c. Initialize any git submodules declared in .gitmodules.
#   4. Print the resolved paths and branch for confirmation.
#
# Environment variables (all optional):
#   CALIB_SRC          — Directory where repos are cloned (default: parent of this repo).
#   CALIB_BRANCH       — Branch to check out (default: feature/camera-lidar-calibration-libs).
#   GITHUB_ORG         — GitHub organization (default: psiori).
#   GIT_URL_SCHEME     — "ssh" (default) or "https" for clone URLs.
#   CALIB_UPDATE_REPOS — Set to "true" to pull latest on existing clones (default: false).
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_SRC="${CALIB_SRC:-$(cd "${CLC_ROOT}/.." && pwd)}"
CALIB_UPDATE_REPOS="${CALIB_UPDATE_REPOS:-false}"
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
    if [[ "${CALIB_UPDATE_REPOS}" == "true" ]]; then
      git -C "${CALIB_SRC}/${repo}" pull --ff-only origin "${CALIB_BRANCH}" || true
    fi
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
