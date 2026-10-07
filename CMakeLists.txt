cmake_minimum_required(VERSION 4.4.0)
project(camera_lidar_calibration VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_CLC_APP "Build Qt QML application" ON)
option(BUILD_WITH_MARCH_NATIVE "Enable -march=native" OFF)
option(CLC_DEV_BUILD "Link against sibling source build trees" ON)

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native -mtune=native)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(PCL REQUIRED COMPONENTS io)
find_package(nlohmann_json REQUIRED)

if(CLC_DEV_BUILD)
  if(NOT TARGET vlcal_align)
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

if(BUILD_CLC_APP)
  find_package(Qt6 REQUIRED COMPONENTS Quick)
  qt_standard_project_setup()
  add_subdirectory(clc_app)
endif()

install(TARGETS clc_core EXPORT camera_lidar_calibrationTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY clc_core/include/ DESTINATION include)
