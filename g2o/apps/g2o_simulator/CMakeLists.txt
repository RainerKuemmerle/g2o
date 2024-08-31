add_executable(g2o_simulator2d_application
  g2o_simulator2d.cpp
)
target_link_libraries(g2o_simulator2d_application simulator_lib types_slam2d_addons types_slam2d core)
set_target_properties(g2o_simulator2d_application PROPERTIES OUTPUT_NAME g2o_simulator2d)

add_executable(g2o_simulator3d_application
  g2o_simulator3d.cpp
)
target_link_libraries(g2o_simulator3d_application simulator_lib types_slam3d_addons types_slam3d types_slam2d_addons types_slam2d core)
set_target_properties(g2o_simulator3d_application PROPERTIES OUTPUT_NAME g2o_simulator3d)

install(TARGETS g2o_simulator2d_application g2o_simulator3d_application
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/apps/g2o_simulator)
