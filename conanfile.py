import os
import sys

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy

sys.path.append(os.path.join(os.path.dirname(__file__), "conan"))
from calib_deps import apply_conan_gtsam_toolchain, apply_macos_toolchain

_EXPORT_EXCLUDES = (
    ".git",
    ".git/*",
    "build",
    "build/*",
    "build_*",
    "build_*/*",
    "install",
    "install/*",
    "cmake-build-*",
    ".cache",
    ".cache/*",
    ".vscode",
    ".idea",
    "__pycache__",
    "__pycache__/*",
    "*.pyc",
    ".DS_Store",
    "conanbuild.sh",
    "conanbuildenv-*",
    "conanrun.sh",
    "conanrunenv-*",
    "deactivate_conanbuild.sh",
    "deactivate_conanrun.sh",
    "compile_commands.json",
    "CMakeUserPresets.json",
    "third_party",
    "third_party/*",
)


class CameraLidarCalibrationConan(ConanFile):
    name = "camera_lidar_calibration"
    version = "0.1.0"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "build_clc_app": [True, False],
        "build_with_march_native": [True, False],
        "clc_dev_build": [True, False],
    }
    default_options = {
        "build_clc_app": True,
        "build_with_march_native": True,
        "clc_dev_build": False,
    }
    def export_sources(self):
        copy(self, "*", self.recipe_folder, self.export_sources_folder, excludes=_EXPORT_EXCLUDES)

    def configure(self):
        self.options["pcl"].with_qt = False
        self.options["pcl"].with_vtk = False

    def requirements(self):
        self.requires("vlcal_align/0.1.0")
        self.requires("glim/1.2.2")
        self.requires("gtsam/4.3a1")
        self.requires("pcl/1.14.1")
        self.requires("ceres-solver/2.2.0")
        self.requires("eigen/3.4.0")
        self.requires("fmt/10.2.1", override=True)
        self.requires("spdlog/1.12.0")
        self.requires("boost/1.83.0")
        self.requires("nlohmann_json/3.11.3")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_CLC_APP"] = self.options.build_clc_app
        tc.variables["BUILD_WITH_MARCH_NATIVE"] = self.options.build_with_march_native
        tc.variables["CLC_DEV_BUILD"] = self.options.clc_dev_build
        apply_macos_toolchain(tc)
        apply_conan_gtsam_toolchain(tc, self)
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "camera_lidar_calibration")
        self.cpp_info.set_property("cmake_target_name", "clc_core")
