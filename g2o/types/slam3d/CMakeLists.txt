ADD_LIBRARY(types_slam3d ${G2O_LIB_TYPE}
  dquat2mat.cpp dquat2mat.h
  isometry3d_mappings.h
  isometry3d_mappings.cpp
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
  parameter_stereo_camera.cpp
  parameter_stereo_camera.h
  edge_se3_pointxyz.cpp
  edge_se3_pointxyz.h
  edge_se3_pointxyz_disparity.cpp
  edge_se3_pointxyz_disparity.h
  edge_se3_pointxyz_depth.cpp
  edge_se3_pointxyz_depth.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  edge_se3_lotsofxyz.cpp
  edge_se3_lotsofxyz.h
  se3quat.h
  se3_ops.h se3_ops.hpp
  edge_pointxyz.cpp edge_pointxyz.h
  types_slam3d.cpp
  types_slam3d.h
)


SET_TARGET_PROPERTIES(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)
TARGET_LINK_LIBRARIES(types_slam3d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam3d opengl_helper ${OPENGL_gl_LIBRARY} )
ENDIF()

ADD_EXECUTABLE(test_isometry3d_mappings test_isometry3d_mappings.cpp)
TARGET_LINK_LIBRARIES(test_isometry3d_mappings types_slam3d)

ADD_EXECUTABLE(test_mat2quat_jacobian test_mat2quat_jacobian.cpp)
TARGET_LINK_LIBRARIES(test_mat2quat_jacobian types_slam3d)

ADD_EXECUTABLE(test_slam3d_jacobian test_slam3d_jacobian.cpp)
TARGET_LINK_LIBRARIES(test_slam3d_jacobian types_slam3d)

INSTALL(TARGETS types_slam3d
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d)
