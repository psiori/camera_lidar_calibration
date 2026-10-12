add_executable(clc_fusion main.cpp)
target_link_libraries(clc_fusion PRIVATE clc_core)
set_target_properties(clc_fusion PROPERTIES OUTPUT_NAME clc_fusion)
install(TARGETS clc_fusion RUNTIME DESTINATION bin)
