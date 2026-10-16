#!/usr/bin/env bash
#
# clone-repos.sh — Clone or update all calibration source repositories.
#
# Dependency repos are checked out at fixed commits below (update SHAs when the stack
# changes, in the same commit as any clc code that needs them). camera_lidar_calibration
# follows CALIB_BRANCH only.
#
# Environment variables (all optional):
#   CALIB_SRC          — Directory where repos are cloned (default: parent of this repo).
#   CALIB_BRANCH       — Branch for camera_lidar_calibration clone (default: feature/camera-lidar-calibration-libs).
#   GITHUB_ORG         — GitHub organization for forked repos (default: psiori).
#   GIT_URL_SCHEME     — "ssh" (default) or "https" for clone URLs.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_SRC="${CALIB_SRC:-$(cd "${CLC_ROOT}/.." && pwd)}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
GIT_URL_SCHEME="${GIT_URL_SCHEME:-ssh}"

# --- pinned dependency commits (edit when bumping stack libs) ---
PIN_gtsam="5db475881f7e40479898ced2e9282f9399fa392d"
PIN_gtsam_points="56d1ded377f9c61fafa33ebbce5f9e54e0fb8aa1"
PIN_glim="da6633b0733237e071182db4688eadc5dd6fc44b"
PIN_direct_visual_lidar_calibration="d08589f7e306d5366c5b1fd0f7c4d4d348c0ff9d"

# name|github_slug|kind|ref   (kind: commit | branch | tag)
repo_specs=(
  "gtsam|${GITHUB_ORG}/gtsam|commit|${PIN_gtsam}"
  "gtsam_points|${GITHUB_ORG}/gtsam_points|commit|${PIN_gtsam_points}"
  "glim|${GITHUB_ORG}/glim|commit|${PIN_glim}"
  "direct_visual_lidar_calibration|${GITHUB_ORG}/direct_visual_lidar_calibration|commit|${PIN_direct_visual_lidar_calibration}"
  "camera_lidar_calibration|${GITHUB_ORG}/camera_lidar_calibration|branch|${CALIB_BRANCH}"
)

repo_url_from_slug() {
  local slug="$1"
  if [[ "${GIT_URL_SCHEME}" == "https" ]]; then
    echo "https://github.com/${slug}.git"
  else
    echo "git@github.com:${slug}.git"
  fi
}

init_submodules() {
  local repo="$1"
  if [[ -f "${CALIB_SRC}/${repo}/.gitmodules" ]]; then
    echo "Initializing submodules for ${repo}..."
    git -C "${CALIB_SRC}/${repo}" submodule update --init --recursive
  fi
}

ensure_origin_url() {
  local dir="$1"
  local expected_url="$2"
  local current_url
  current_url="$(git -C "${dir}" remote get-url origin 2>/dev/null || true)"
  if [[ -n "${current_url}" && "${current_url}" != "${expected_url}" ]]; then
    echo "  origin -> ${expected_url} (was ${current_url})"
    git -C "${dir}" remote set-url origin "${expected_url}"
  fi
}

checkout_ref() {
  local dir="$1"
  local kind="$2"
  local ref="$3"

  if [[ "${kind}" == "commit" ]]; then
    echo "  checkout ${ref}"
    git -C "${dir}" checkout --detach "${ref}"
    return
  fi

  if [[ "${kind}" == "tag" ]]; then
    git -C "${dir}" checkout --detach "${ref}"
    return
  fi

  git -C "${dir}" checkout "${ref}" 2>/dev/null \
    || git -C "${dir}" checkout -B "${ref}" "origin/${ref}"
}

clone_or_update_spec() {
  local name slug kind ref url dir
  IFS='|' read -r name slug kind ref <<< "$1"
  url="$(repo_url_from_slug "${slug}")"
  dir="${CALIB_SRC}/${name}"

  if [[ -d "${dir}/.git" ]]; then
    echo "Updating ${name} (${kind} ${ref})..."
    ensure_origin_url "${dir}" "${url}"
    git -C "${dir}" fetch origin --tags
    checkout_ref "${dir}" "${kind}" "${ref}"
  else
    echo "Cloning ${name} (${kind} ${ref})..."
    if [[ "${kind}" == "branch" ]]; then
      git clone --recurse-submodules --branch "${ref}" "${url}" "${dir}"
    else
      git clone --recurse-submodules "${url}" "${dir}"
      checkout_ref "${dir}" "${kind}" "${ref}"
    fi
  fi
  init_submodules "${name}"
}

mkdir -p "${CALIB_SRC}"

echo "CALIB_SRC=${CALIB_SRC}"
echo "CALIB_BRANCH=${CALIB_BRANCH} (camera_lidar_calibration only)"
echo "GITHUB_ORG=${GITHUB_ORG}"

for spec in "${repo_specs[@]}"; do
  clone_or_update_spec "${spec}"
done

echo "Repositories ready under ${CALIB_SRC}"
