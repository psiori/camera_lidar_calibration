#!/usr/bin/env bash
#
# clone-repos.sh — Clone or update all calibration source repositories.
#
# What this script does, step by step:
#   1. Resolve CALIB_SRC (parent directory for all repos) and git settings.
#   2. Create CALIB_SRC if it does not exist.
#   3. For each repo in the stack:
#        a. If already cloned: ensure origin URL, fetch, check out the configured ref.
#        b. If not cloned: git clone (branch repos use --branch; tag repos clone then detach).
#        c. Initialize any git submodules declared in .gitmodules.
#   4. Print the resolved paths and branch for confirmation.
#
# All stack repos are psiori forks on CALIB_BRANCH (gtsam includes Conan packaging). Only
# use a different slug/kind/ref in repo_specs if a repo is intentionally unpinned.
#
# Environment variables (all optional):
#   CALIB_SRC          — Directory where repos are cloned (default: parent of this repo).
#   CALIB_BRANCH       — Branch for forked repos (default: feature/camera-lidar-calibration-libs).
#   GITHUB_ORG         — GitHub organization for forked repos (default: psiori).
#   GIT_URL_SCHEME     — "ssh" (default) or "https" for clone URLs.
#   CALIB_UPDATE_REPOS — Pull latest on existing branch checkouts when remote is ahead (default: true).
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_SRC="${CALIB_SRC:-$(cd "${CLC_ROOT}/.." && pwd)}"
CALIB_UPDATE_REPOS="${CALIB_UPDATE_REPOS:-true}"
CALIB_BRANCH="${CALIB_BRANCH:-feature/camera-lidar-calibration-libs}"
GITHUB_ORG="${GITHUB_ORG:-psiori}"
GIT_URL_SCHEME="${GIT_URL_SCHEME:-ssh}"

# name|github_slug|kind|ref   (kind: tag | branch)
repo_specs=(
  "gtsam|${GITHUB_ORG}/gtsam|branch|${CALIB_BRANCH}"
  "gtsam_points|${GITHUB_ORG}/gtsam_points|branch|${CALIB_BRANCH}"
  "glim|${GITHUB_ORG}/glim|branch|${CALIB_BRANCH}"
  "direct_visual_lidar_calibration|${GITHUB_ORG}/direct_visual_lidar_calibration|branch|${CALIB_BRANCH}"
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

  if [[ "${kind}" == "tag" ]]; then
    git -C "${dir}" checkout --detach "${ref}"
    return
  fi

  git -C "${dir}" checkout "${ref}" 2>/dev/null \
    || git -C "${dir}" checkout -B "${ref}" "origin/${ref}"
  if [[ "${CALIB_UPDATE_REPOS}" == "true" ]]; then
    git -C "${dir}" pull --ff-only origin "${ref}" || true
  fi
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
      git -C "${dir}" checkout --detach "${ref}"
    fi
  fi
  init_submodules "${name}"
}

mkdir -p "${CALIB_SRC}"

echo "CALIB_SRC=${CALIB_SRC}"
echo "CALIB_BRANCH=${CALIB_BRANCH} (forked repos only)"
echo "GITHUB_ORG=${GITHUB_ORG}"

for spec in "${repo_specs[@]}"; do
  clone_or_update_spec "${spec}"
done

echo "Repositories ready under ${CALIB_SRC}"
