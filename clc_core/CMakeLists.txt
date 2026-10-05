add_library(clc_core SHARED
  src/pcd_io.cpp
  src/session.cpp
  src/calibration_service.cpp
  src/fusion_service.cpp
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
if(TARGET glim::cloud_fusion)
  set(_clc_fusion_target glim::cloud_fusion)
else()
  set(_clc_fusion_target glim_cloud_fusion)
endif()
target_link_libraries(clc_core PUBLIC
  ${_clc_vlcal_target}
  ${_clc_fusion_target}
  Eigen3::Eigen
  ${PCL_IO_LIBRARIES}
)
target_include_directories(clc_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
