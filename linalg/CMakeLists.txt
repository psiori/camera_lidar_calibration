add_library(clc_linalg INTERFACE)
target_include_directories(clc_linalg INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(DIRECTORY include/ DESTINATION include)
