#!/usr/bin/env bash
# Install Homebrew packages used as prebuilt system dependencies on macOS.
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  exit 0
fi

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is required on macOS for opencv, pcl, qt, ceres, and libomp."
  echo "Install from https://brew.sh then re-run."
  exit 1
fi

BREW_PACKAGES=(
  opencv
  qt@6
  libomp
  ninja
)

missing=()
for pkg in "${BREW_PACKAGES[@]}"; do
  if ! brew list --formula "${pkg}" >/dev/null 2>&1; then
    missing+=("${pkg}")
  fi
done

if ((${#missing[@]} > 0)); then
  echo "Installing Homebrew dependencies: ${missing[*]}"
  brew install "${missing[@]}"
fi

BREW_PREFIX="$(brew --prefix)"
echo "Homebrew prefix: ${BREW_PREFIX}"
echo "Qt6 CMake package: $(brew --prefix qt@6 2>/dev/null || echo missing)"
