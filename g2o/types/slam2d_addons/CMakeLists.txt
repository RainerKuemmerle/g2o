ADD_LIBRARY(types_slam2d_addons ${G2O_LIB_TYPE}
  line_2d.h
  types_slam2d_addons.cpp types_slam2d_addons.h
  vertex_segment2d.cpp         vertex_segment2d.h
  edge_se2_segment2d.cpp       edge_se2_segment2d.h
  edge_se2_segment2d_line.cpp       edge_se2_segment2d_line.h
  edge_se2_segment2d_pointLine.cpp       edge_se2_segment2d_pointLine.h
  vertex_line2d.cpp            vertex_line2d.h
  edge_se2_line2d.cpp          edge_se2_line2d.h
  edge_line2d_pointxy.cpp          edge_line2d_pointxy.h
  edge_line2d.cpp          edge_line2d.h
  g2o_types_slam2d_addons_api.h
)

SET_TARGET_PROPERTIES(types_slam2d_addons PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d_addons)

TARGET_LINK_LIBRARIES(types_slam2d_addons types_slam2d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam2d_addons ${OPENGL_gl_LIBRARY})
ENDIF()

INSTALL(TARGETS types_slam2d_addons
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam2d_addons)
