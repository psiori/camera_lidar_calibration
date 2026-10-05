from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


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
    exports_sources = "*"

    def requirements(self):
        self.requires("vlcal_align/0.1.0")
        self.requires("glim/1.2.2")
        self.requires("pcl/1.14.1")
        self.requires("opencv/4.10.0")
        self.requires("eigen/3.4.0")
        self.requires("nlohmann_json/3.11.3")
        if self.options.build_clc_app:
            self.requires("qt/6.7.3")

    def configure(self):
        self.options["opencv"].with_ffmpeg = False
        self.options["opencv"].with_gtk = False

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_CLC_APP"] = self.options.build_clc_app
        tc.variables["BUILD_WITH_MARCH_NATIVE"] = self.options.build_with_march_native
        tc.variables["CLC_DEV_BUILD"] = self.options.clc_dev_build
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
