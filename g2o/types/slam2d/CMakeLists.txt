add_library(types_slam2d ${G2O_LIB_TYPE}
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
  edge_xy_prior.cpp		edge_xy_prior.h
  g2o_types_slam2d_api.h
)

set_target_properties(types_slam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d)
if (APPLE)
  set_target_properties(types_slam2d PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_link_libraries(types_slam2d core)
if(G2O_HAVE_OPENGL)
  target_link_libraries(types_slam2d opengl_helper ${OPENGL_gl_LIBRARY} )
endif()

install(TARGETS types_slam2d
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/slam2d)
