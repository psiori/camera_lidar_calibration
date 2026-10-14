cmake_minimum_required(VERSION 4.4.0)
project(camera_lidar_calibration VERSION 0.1.0 LANGUAGES CXX)

enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_CLC_APP "Build Qt QML application" ON)
option(BUILD_CLC_FUSION_CLI "Build clc_fusion command-line fusion tool" ON)
option(BUILD_WITH_MARCH_NATIVE "Enable -march=native" OFF)
option(CLC_DEV_BUILD "Link against sibling source build trees" ON)

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native -mtune=native)
endif()

find_package(Eigen3 REQUIRED)

add_subdirectory(linalg)

# Homebrew opencv@4 only (see BUILD.md). Avoid Conan static opencv on the app link line.
if(NOT OpenCV_DIR)
  if(EXISTS "/opt/homebrew/opt/opencv@4/lib/cmake/opencv4/OpenCVConfig.cmake")
    set(OpenCV_DIR "/opt/homebrew/opt/opencv@4/lib/cmake/opencv4" CACHE PATH "Homebrew opencv@4")
  elseif(EXISTS "/usr/local/opt/opencv@4/lib/cmake/opencv4/OpenCVConfig.cmake")
    set(OpenCV_DIR "/usr/local/opt/opencv@4/lib/cmake/opencv4" CACHE PATH "Homebrew opencv@4")
  endif()
endif()
if(OpenCV_DIR)
  find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs
    PATHS "${OpenCV_DIR}" NO_DEFAULT_PATH NO_CMAKE_PACKAGE_REGISTRY)
else()
  find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
endif()

find_package(PCL REQUIRED COMPONENTS io)
find_package(nlohmann_json REQUIRED)

if(CLC_DEV_BUILD)
  if(NOT TARGET vlcal_align)
    set(BUILD_VLCAL_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(
      "${CMAKE_CURRENT_SOURCE_DIR}/../direct_visual_lidar_calibration"
      "${CMAKE_BINARY_DIR}/direct_visual_lidar_calibration"
      EXCLUDE_FROM_ALL)
  endif()
  if(NOT TARGET glim)
    set(BUILD_WITH_VIEWER OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_CUDA OFF CACHE BOOL "" FORCE)
    set(BUILD_GLIM_CLOUD_FUSION ON CACHE BOOL "" FORCE)
    set(BUILD_GLIM_CLOUD_FUSION_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../glim" "${CMAKE_BINARY_DIR}/glim" EXCLUDE_FROM_ALL)
  endif()
else()
  find_package(direct_visual_lidar_calibration REQUIRED)
  find_package(glim REQUIRED)
  find_package(glim_cloud_fusion REQUIRED)
endif()

add_subdirectory(clc_core)

if(BUILD_CLC_FUSION_CLI)
  add_subdirectory(tools/clc_fusion)
endif()

if(BUILD_CLC_APP)
  find_package(Qt6 REQUIRED COMPONENTS Quick Widgets Qml Gui Core ShaderTools Test)
  qt_standard_project_setup()
  add_subdirectory(psiviz)
  add_subdirectory(clc_app)
endif()

install(TARGETS clc_core EXPORT camera_lidar_calibrationTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY clc_core/include/ DESTINATION include)
