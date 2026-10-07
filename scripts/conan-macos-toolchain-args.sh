#!/usr/bin/env bash
#
# conan-macos-toolchain-args.sh — Emit Conan toolchain config for macOS builds.
#
# What this script does, step by step:
#   1. Exit with no output on non-macOS systems (Linux builds do not need this).
#   2. Run conan/emit_toolchain_conf.py to generate a toolchain configuration string
#      that points Conan at Homebrew OpenCV, OpenMP (libomp), and Qt6 paths.
#   3. Print the result as null-delimited Conan -c arguments for the caller to consume.
#
# This script is sourced by install-deps.sh and build-app.sh via process substitution.
# It produces no output on Linux; on macOS it supplies -c <toolchain_conf> to conan.
#
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONF="$(python3 "${CLC_ROOT}/conan/emit_toolchain_conf.py")"

if [[ -n "${CONF}" ]]; then
  printf '%s\0%s\0' "-c" "${CONF}"
fi
