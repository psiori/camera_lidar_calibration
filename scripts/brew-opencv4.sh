#!/usr/bin/env bash
#
# brew-opencv4.sh — Homebrew opencv@4 paths for the calibration app shell.
#
# OpenCV 5 (Homebrew formula "opencv") is incompatible with vlcal and clc_core.
# Source this file from other scripts; do not execute directly.
#

brew_opencv4_prefix() {
  if ! command -v brew >/dev/null 2>&1; then
    return 1
  fi
  brew --prefix opencv@4 2>/dev/null
}

# Prints the directory containing OpenCVConfig.cmake for opencv@4.
brew_opencv4_cmake_dir() {
  local prefix dir
  prefix="$(brew_opencv4_prefix)" || return 1
  dir="${prefix}/lib/cmake/opencv4"
  if [[ -d "${dir}" ]]; then
    echo "${dir}"
    return 0
  fi
  return 1
}
