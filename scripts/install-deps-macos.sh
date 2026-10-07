#!/usr/bin/env bash
#
# install-deps-macos.sh — macOS convenience entry point for install-deps.sh.
#
# What this script does:
#   1. Resolve the scripts/ directory relative to this file.
#   2. Delegate to install-deps.sh, which handles Conan builds and Homebrew deps.
#
# Use this when you want an explicit macOS-named command; behavior is identical
# to install-deps.sh (which already detects Darwin and applies macOS toolchains).
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/install-deps.sh"
