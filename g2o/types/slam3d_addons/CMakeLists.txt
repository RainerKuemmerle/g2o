ADD_LIBRARY(types_slam3d_addons ${G2O_LIB_TYPE}
  vertex_se3_euler.cpp
  vertex_se3_euler.h
  edge_se3_euler.cpp
  edge_se3_euler.h
  vertex_plane.cpp
  vertex_plane.h
  edge_se3_plane_calib.cpp
  edge_se3_plane_calib.h
  line3d.cpp line3d.h
  vertex_line3d.cpp vertex_line3d.h
  edge_se3_line.cpp edge_se3_line.h
  edge_plane.cpp edge_plane.h
  edge_se3_calib.cpp edge_se3_calib.h
  types_slam3d_addons.cpp
  types_slam3d_addons.h
)


SET_TARGET_PROPERTIES(types_slam3d_addons PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d_addons)

TARGET_LINK_LIBRARIES(types_slam3d_addons types_slam3d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam3d_addons ${OPENGL_gl_LIBRARY} )
ENDIF()

INSTALL(TARGETS types_slam3d_addons
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d_addons)
