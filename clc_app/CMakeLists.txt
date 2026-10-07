qt_add_executable(clc_app
  main.cpp
  app_controller.cpp
)
qt_add_qml_module(clc_app
  URI CameraLidarCalibration
  VERSION 1.0
  QML_FILES Main.qml
)
if(TARGET opencv_imgcodecs)
  set(_clc_opencv_imgcodecs opencv_imgcodecs)
elseif(TARGET OpenCV::imgcodecs)
  set(_clc_opencv_imgcodecs OpenCV::imgcodecs)
else()
  set(_clc_opencv_imgcodecs ${OpenCV_LIBS})
endif()
target_link_libraries(clc_app PRIVATE
  Qt6::Quick
  Qt6::Qml
  Qt6::QmlModels
  Qt6::Gui
  Qt6::Core
  clc_core
  ${_clc_opencv_imgcodecs}
)
target_include_directories(clc_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
