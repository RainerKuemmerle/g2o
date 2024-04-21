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
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)
