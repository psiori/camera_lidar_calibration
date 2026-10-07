#!/usr/bin/env bash
#
# build-app-macos.sh — macOS convenience entry point for build-app.sh.
#
# What this script does:
#   1. Resolve the scripts/ directory relative to this file.
#   2. Delegate to build-app.sh, which configures and builds clc_app with Conan + Homebrew.
#
# Use this when you want an explicit macOS-named command; behavior is identical
# to build-app.sh (which already detects Darwin and applies macOS toolchains).
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/build-app.sh"
