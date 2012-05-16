ADD_LIBRARY(types_slam3d_new ${G2O_LIB_TYPE}
  isometry3d_mappings.h
  isometry3d_gradients.cpp
  isometry3d_gradients.h
  vertex_se3.cpp
  vertex_se3.h
  parameter_se3_offset.cpp
  parameter_se3_offset.h
  edge_se3.cpp
  edge_se3.h
  edge_se3_offset.cpp
  edge_se3_offset.h
  vertex_pointxyz.cpp
  vertex_pointxyz.h
  parameter_camera.cpp
  parameter_camera.h
  edge_se3_pointxyz.cpp
  edge_se3_pointxyz.h
  edge_se3_pointxyz_disparity.cpp
  edge_se3_pointxyz_disparity.h
  edge_se3_pointxyz_depth.cpp
  edge_se3_pointxyz_depth.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  types_slam3d_new.cpp
  types_slam3d_new.h
)

ADD_DEPENDENCIES(types_slam3d_new isometry3d_mappings.cpp)

SET_TARGET_PROPERTIES(types_slam3d_new PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d_new)

TARGET_LINK_LIBRARIES(types_slam3d_new ${OPENGL_gl_LIBRARY} core)

ADD_EXECUTABLE(test_isometry3d_mappings test_isometry3d_mappings.cpp)

TARGET_LINK_LIBRARIES(test_isometry3d_mappings types_slam3d_new)

INSTALL(TARGETS types_slam3d_new
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d_new)


