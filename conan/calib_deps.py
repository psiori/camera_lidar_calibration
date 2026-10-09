"""Calibration-stack Conan integration (camera_lidar_calibration only).

Applied via native-macos profile conf from install-deps.sh / build-app.sh, and
from this repo's conanfile when building the app. Library repos must not import this.
"""

import os
import shutil
import subprocess

# Qt and Homebrew opencv@4 for the app shell are passed in scripts/build-app.sh.
# Do not set OpenCV_DIR here: Conan packages (e.g. vlcal_align) must use OpenCV
# 4.x so installed CMake targets do not embed brittle Homebrew paths.
_HOMEBREW_CMAKE_HINTS = (
    ("Qt6_DIR", "opt/qt@6/lib/cmake/Qt6"),
)


def _homebrew_prefix():
    brew = shutil.which("brew")
    if not brew:
        return None
    try:
        return subprocess.check_output([brew, "--prefix"], text=True).strip()
    except (subprocess.CalledProcessError, OSError):
        return None


def macos_toolchain_extra_variables():
    """CMake toolchain variables for the hybrid Homebrew + Conan stack on macOS."""
    if os.uname().sysname != "Darwin":
        return {}

    prefix = _homebrew_prefix()
    if not prefix:
        return {}

    variables = {
        "CMAKE_IGNORE_PREFIX_PATH": f"{prefix};/usr/local",
    }

    for var, rel in _HOMEBREW_CMAKE_HINTS:
        path = os.path.join(prefix, rel)
        if os.path.isdir(path):
            variables[var] = path

    libomp = os.path.join(prefix, "opt", "libomp")
    omp_h = os.path.join(libomp, "include", "omp.h")
    omp_lib = os.path.join(libomp, "lib", "libomp.dylib")
    if os.path.isfile(omp_h) and os.path.isfile(omp_lib):
        omp_inc = os.path.join(libomp, "include")
        variables["OpenMP_CXX_FLAGS"] = "-Xclang -fopenmp"
        variables["OpenMP_C_FLAGS"] = "-Xclang -fopenmp"
        variables["OpenMP_CXX_LIB_NAMES"] = "omp"
        variables["OpenMP_C_LIB_NAMES"] = "omp"
        variables["OpenMP_omp_LIBRARY"] = omp_lib
        variables["OpenMP_CXX_INCLUDE_DIR"] = omp_inc
        variables["OpenMP_C_INCLUDE_DIR"] = omp_inc

    return variables


def apply_macos_toolchain(tc):
    for key, value in macos_toolchain_extra_variables().items():
        tc.variables[key] = value


def apply_conan_gtsam_toolchain(tc, conanfile):
    """GTSAM uses cmake_find_mode=none; point CMake at the Conan package config."""
    try:
        gtsam = conanfile.dependencies["gtsam"]
    except (KeyError, AttributeError):
        return
    gtsam_dir = os.path.join(gtsam.package_folder, "lib", "cmake", "GTSAM")
    if os.path.isdir(gtsam_dir):
        tc.variables["GTSAM_DIR"] = gtsam_dir
