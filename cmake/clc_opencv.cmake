# OpenCV for clc_core: Conan opencv:: from vlcal_align/glim (transitive), or Homebrew OpenCV::.

macro(clc_ensure_opencv_found)
  if(NOT TARGET opencv::opencv_core
      AND NOT TARGET OpenCV::opencv_core
      AND NOT TARGET OpenCV::core)
    find_package(opencv CONFIG QUIET)
  endif()
  if(NOT TARGET opencv::opencv_core
      AND NOT TARGET OpenCV::opencv_core
      AND NOT TARGET OpenCV::core)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
  endif()
endmacro()

macro(clc_target_link_opencv target)
  clc_ensure_opencv_found()
  if(TARGET opencv::opencv_core)
    target_link_libraries(${target} PUBLIC
      opencv::opencv_core
      opencv::opencv_imgproc
      opencv::opencv_imgcodecs)
  elseif(TARGET OpenCV::opencv_core)
    target_link_libraries(${target} PUBLIC
      OpenCV::opencv_core
      OpenCV::opencv_imgproc
      OpenCV::opencv_imgcodecs)
  elseif(TARGET OpenCV::core)
    target_link_libraries(${target} PUBLIC
      OpenCV::core
      OpenCV::imgproc
      OpenCV::imgcodecs)
  else()
    if(NOT OpenCV_INCLUDE_DIRS)
      message(FATAL_ERROR "OpenCV found but no imported targets or include dirs")
    endif()
    target_include_directories(${target} SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PUBLIC ${OpenCV_LIBS})
  endif()
endmacro()
