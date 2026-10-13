qt_add_executable(clc_app
  main.cpp
  app_controller.cpp
  range_image_item.cpp
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
    CameraImageItem.qml
    PointCloudViewerPane.qml
    AddDatasetDialog.qml
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
option(CLC_DEV_ADD_DATASET_DEFAULTS "Pre-fill Add Dataset dialog with dev sample paths" ON)
if(CLC_DEV_ADD_DATASET_DEFAULTS)
  target_compile_definitions(clc_app PRIVATE CLC_DEV_ADD_DATASET_DEFAULTS)
endif()
qt_import_qml_plugins(clc_app)

qt_add_executable(clc_mask_editor_polygon_test
  tests/mask_editor_polygon_test.cpp
  app_controller.cpp
)
target_compile_definitions(clc_mask_editor_polygon_test PRIVATE CLC_APP_TESTING)
qt_add_qml_module(clc_mask_editor_polygon_test
  URI CameraLidarCalibrationTest
  VERSION 1.0
  QML_FILES
    MaskEditorWindow.qml
    ColormapRangeSlider.qml
  SOURCES
    range_image_item.cpp
)
target_link_libraries(clc_mask_editor_polygon_test PRIVATE
  Qt6::Quick
  Qt6::Widgets
  Qt6::Qml
  Qt6::QmlModels
  Qt6::Gui
  Qt6::Core
  Qt6::Test
  clc_core
  psiviz
  psivizplugin
)
target_include_directories(clc_mask_editor_polygon_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
qt_import_qml_plugins(clc_mask_editor_polygon_test)
set_target_properties(clc_mask_editor_polygon_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/clc_app"
)
add_test(NAME clc_mask_editor_polygon_test COMMAND clc_mask_editor_polygon_test)
