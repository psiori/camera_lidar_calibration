qt_add_executable(clc_app
  main.cpp
  app_controller.cpp
)
qt_add_qml_module(clc_app
  URI CameraLidarCalibration
  VERSION 1.0
  QML_FILES Main.qml
)
target_link_libraries(clc_app PRIVATE Qt6::Quick clc_core)
target_include_directories(clc_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
