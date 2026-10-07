#true!/usr/bin/env bash
# Print Conan -c arguments for the calibration-stack macOS toolchain (or nothing).
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
