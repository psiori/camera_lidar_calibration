from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class GtsamConan(ConanFile):
    name = "gtsam"
    version = "4.3a1"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "build_with_march_native": [True, False],
    }
    default_options = {"shared": True, "fPIC": True, "build_with_march_native": True}

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("boost/1.83.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["GTSAM_BUILD_EXAMPLES_ALWAYS"] = False
        tc.variables["GTSAM_BUILD_TESTS"] = False
        tc.variables["GTSAM_WITH_TBB"] = False
        tc.variables["GTSAM_BUILD_WITH_MARCH_NATIVE"] = self.options.build_with_march_native
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
        self.cpp_info.set_property("cmake_file_name", "GTSAM")
        self.cpp_info.set_property("cmake_target_name", "gtsam::gtsam")
