ADD_LIBRARY(types_slam2d ${G2O_LIB_TYPE}
  se2.h
  edge_se2_pointxy_bearing.h  edge_se2_prior.h
  edge_se2.cpp                  edge_se2_pointxy_calib.cpp  types_slam2d.cpp
  edge_se2.h                    edge_se2_pointxy_calib.h    vertex_point_xy.cpp
  edge_se2_pointxy.cpp        vertex_point_xy.h
  edge_se2_pointxy.h          vertex_se2.cpp
  edge_se2_pointxy_bearing.cpp  edge_se2_prior.cpp          vertex_se2.h
  parameter_se2_offset.cpp     parameter_se2_offset.h
  edge_se2_offset.cpp          edge_se2_offset.h
  edge_se2_pointxy_offset.cpp  edge_se2_pointxy_offset.h
  edge_se2_xyprior.cpp edge_se2_xyprior.h
  edge_pointxy.cpp             edge_pointxy.h
  edge_se2_twopointsxy.cpp	edge_se2_twopointsxy.h
  edge_se2_lotsofxy.cpp		edge_se2_lotsofxy.h
  g2o_types_slam2d_api.h
)

SET_TARGET_PROPERTIES(types_slam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d)

TARGET_LINK_LIBRARIES(types_slam2d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam2d opengl_helper ${OPENGL_gl_LIBRARY} )
ENDIF()

INSTALL(TARGETS types_slam2d
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam2d)
