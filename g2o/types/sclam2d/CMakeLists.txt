ADD_LIBRARY(types_sclam2d ${G2O_LIB_TYPE}
  edge_se2_sensor_calib.cpp edge_se2_sensor_calib.h
  vertex_odom_differential_params.cpp vertex_odom_differential_params.h
  edge_se2_odom_differential_calib.cpp edge_se2_odom_differential_calib.h
  odometry_measurement.cpp odometry_measurement.h
  types_sclam2d.cpp types_sclam2d.h
  g2o_types_sclam2d_api.h
)

SET_TARGET_PROPERTIES(types_sclam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sclam2d)

TARGET_LINK_LIBRARIES(types_sclam2d types_slam2d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_sclam2d ${OPENGL_gl_LIBRARY})
ENDIF()

INSTALL(TARGETS types_sclam2d
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/sclam2d)
