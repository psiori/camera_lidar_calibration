qt_add_executable(clc_app
  main.cpp
  app_controller.cpp
)
qt_add_qml_module(clc_app
  URI CameraLidarCalibration
  VERSION 1.0
  QML_FILES
    Main.qml
    Expansion.qml
    MaskEditorWindow.qml
    ViewerOverlay.qml
    ColormapRangeSlider.qml
  SOURCES
    range_image_item.cpp
)
target_link_libraries(clc_app PRIVATE
  Qt6::Quick
  Qt6::Widgets
  Qt6::Qml
  Qt6::QmlModels
  Qt6::Gui
  Qt6::Core
  clc_core
  psiviz
  psivizplugin
)
target_include_directories(clc_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
qt_import_qml_plugins(clc_app)
