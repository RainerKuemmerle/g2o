add_library(types_slam3d ${G2O_LIB_TYPE}
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
  edge_se3_xyzprior.cpp
  edge_se3_xyzprior.h
  edge_se3_lotsofxyz.cpp
  edge_se3_lotsofxyz.h
  edge_xyz_prior.cpp
  edge_xyz_prior.h
  se3quat.h
  se3_ops.h se3_ops.hpp
  edge_pointxyz.cpp edge_pointxyz.h
  types_slam3d.cpp
  types_slam3d.h
)


set_target_properties(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)
if (APPLE)
  set_target_properties(types_slam3d PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_link_libraries(types_slam3d core)
if(G2O_HAVE_OPENGL)
  target_link_libraries(types_slam3d opengl_helper ${OPENGL_gl_LIBRARY} )
endif()

install(TARGETS types_slam3d
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/slam3d)
