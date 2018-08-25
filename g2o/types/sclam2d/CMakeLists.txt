add_library(types_sclam2d ${G2O_LIB_TYPE}
  edge_se2_sensor_calib.cpp edge_se2_sensor_calib.h
  vertex_odom_differential_params.cpp vertex_odom_differential_params.h
  edge_se2_odom_differential_calib.cpp edge_se2_odom_differential_calib.h
  odometry_measurement.cpp odometry_measurement.h
  types_sclam2d.cpp types_sclam2d.h
  g2o_types_sclam2d_api.h
)

set_target_properties(types_sclam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sclam2d)
if (APPLE)
  set_target_properties(types_sclam2d PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
target_link_libraries(types_sclam2d types_slam2d core)
if(G2O_HAVE_OPENGL)
  target_link_libraries(types_sclam2d ${OPENGL_gl_LIBRARY})
endif()

install(TARGETS types_sclam2d
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/sclam2d)
