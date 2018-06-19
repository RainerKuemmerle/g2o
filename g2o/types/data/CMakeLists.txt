ADD_LIBRARY(types_data ${G2O_LIB_TYPE}
  types_data.cpp
  robot_data.cpp robot_data.h
  vertex_tag.cpp vertex_tag.h
  vertex_ellipse.cpp vertex_ellipse.h
  laser_parameters.cpp laser_parameters.h
  raw_laser.cpp raw_laser.h
  robot_laser.cpp robot_laser.h
  data_queue.cpp data_queue.h
  g2o_types_data_api.h
)

SET_TARGET_PROPERTIES(types_data PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_data)

TARGET_LINK_LIBRARIES(types_data core types_slam2d)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_data freeglut_minimal opengl_helper)
ENDIF()

INSTALL(TARGETS types_data
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/data)
