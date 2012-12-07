ADD_LIBRARY(types_slam2d_addons ${G2O_LIB_TYPE}
  types_slam2d_addons.cpp types_slam2d_addons.h
  vertex_segment2d.cpp         vertex_segment2d.h
  edge_se2_segment2d.cpp       edge_se2_segment2d.h
  edge_se2_segment2d_line.cpp       edge_se2_segment2d_line.h
  edge_se2_segment2d_pointLine.cpp       edge_se2_segment2d_pointLine.h
  vertex_line2d.cpp            vertex_line2d.h
  edge_se2_line2d.cpp          edge_se2_line2d.h
  edge_line2d_pointxy.cpp          edge_line2d_pointxy.h
  g2o_types_slam2d_addons_api.h
  
)

SET_TARGET_PROPERTIES(types_slam2d_addons PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d_addons)

TARGET_LINK_LIBRARIES(types_slam2d_addons types_slam2d core ${OPENGL_gl_LIBRARY})

# ADD_EXECUTABLE(g2o_simulator2d_segment_application
#   simulator2d_segment.cpp
# )
# TARGET_LINK_LIBRARIES(g2o_simulator2d_segment_application g2o_simulator_library types_slam2d_segment types_slam3d types_slam2d core)
# SET_TARGET_PROPERTIES(g2o_simulator2d_segment_application PROPERTIES OUTPUT_NAME g2o_simulator2d_segment${EXE_POSTFIX})


# ADD_EXECUTABLE(convertSegmentLine_application convertSegmentLine.cpp)

# TARGET_LINK_LIBRARIES(convertSegmentLine_application g2o_simulator_library types_slam2d_segment types_slam3d types_slam2d core)
# SET_TARGET_PROPERTIES(convertSegmentLine_application PROPERTIES OUTPUT_NAME convertSegmentLine${EXE_POSTFIX})


INSTALL(TARGETS #types_slam2d_segment g2o_simulator2d_segment_application
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam2d_addons)
