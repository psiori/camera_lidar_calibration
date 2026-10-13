add_library(clc_core SHARED
  src/pcd_io.cpp
  src/session.cpp
  src/calibration_service.cpp
  src/fusion_service.cpp
  src/camera_intrinsics_io.cpp
  src/scan_geometry.cpp
  src/dataset_util.cpp
  src/colormap.cpp
  src/range_image.cpp
  src/point_color_model.cpp
)
target_include_directories(clc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(TARGET direct_visual_lidar_calibration::vlcal_align)
  set(_clc_vlcal_target direct_visual_lidar_calibration::vlcal_align)
else()
  set(_clc_vlcal_target vlcal_align)
endif()
if(TARGET glim::glim_cloud_fusion)
  set(_clc_fusion_target glim::glim_cloud_fusion)
elseif(TARGET glim::cloud_fusion)
  set(_clc_fusion_target glim::cloud_fusion)
else()
  set(_clc_fusion_target glim_cloud_fusion)
endif()
if(TARGET PCL::io)
  set(_clc_pcl_target PCL::io)
else()
  set(_clc_pcl_target ${PCL_IO_LIBRARIES})
endif()
target_link_libraries(clc_core PUBLIC
  ${_clc_vlcal_target}
  ${_clc_fusion_target}
  Eigen3::Eigen
  ${_clc_pcl_target}
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
)
if(PCL_INCLUDE_DIRS)
  target_include_directories(clc_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
endif()

option(BUILD_CLC_CORE_TESTS "Build clc_core unit tests" ON)
if(BUILD_CLC_CORE_TESTS)
  add_executable(clc_core_tests tests/clc_core_tests.cpp)
  target_link_libraries(clc_core_tests PRIVATE clc_core)
  add_test(NAME clc_core_tests COMMAND clc_core_tests)
endif()
