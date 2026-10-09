find_package(Qt6 REQUIRED COMPONENTS Quick ShaderTools GuiPrivate Test)
find_package(Eigen3 REQUIRED)

qt_add_library(psiviz STATIC)

qt_add_qml_module(psiviz
  URI Psiviz
  VERSION 1.0
  QML_FILES qml/PointCloudView.qml
  SOURCES
    src/colormap.cpp
    src/trackball_camera.cpp
    src/depth_picker.cpp
    src/pose_gizmo.cpp
    src/viewer_input.cpp
    src/point_cloud_renderer.cpp
    src/point_cloud_item.cpp
)

target_sources(psiviz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/psiviz/point_cloud_item.hpp)

qt6_add_shaders(psiviz "psiviz_shaders"
  PREFIX "/psiviz"
  FILES
    shaders/pointcloud.vert
    shaders/pointcloud.frag
    shaders/gizmo.vert
    shaders/gizmo.frag
    shaders/pick.frag
)

target_include_directories(psiviz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/psiviz
)

target_link_libraries(psiviz
  PUBLIC
    Qt6::Quick
    Qt6::Gui
    Eigen3::Eigen
  PRIVATE
    Qt6::ShaderTools
    Qt6::GuiPrivate
)

install(TARGETS psiviz EXPORT camera_lidar_calibrationTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

qt_add_executable(psiviz_scene_test tests/point_cloud_scene_test.cpp)
target_link_libraries(psiviz_scene_test PRIVATE psiviz psivizplugin Qt6::Quick Qt6::Test)

qt_add_executable(psiviz_trackball_test tests/trackball_camera_test.cpp)
target_link_libraries(psiviz_trackball_test PRIVATE psiviz Qt6::Test)
