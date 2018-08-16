add_library(deprecated_types_slam3d ${G2O_LIB_TYPE}
  se3quat_gradients.cpp
  se3quat_gradients.h
  vertex_se3_quat.cpp
  vertex_se3_quat.h
  edge_se3_quat.cpp
  edge_se3_quat.h
  vertex_pointxyz.cpp
  vertex_pointxyz.h
  parameter_se3_offset.cpp
  parameter_se3_offset.h
  edge_se3_offset.cpp
  edge_se3_offset.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  edge_se3_pointxyz.cpp
  edge_se3_pointxyz.h
  parameter_camera.cpp
  parameter_camera.h
  edge_se3_pointxyz_disparity.cpp
  edge_se3_pointxyz_disparity.h
  edge_se3_pointxyz_depth.cpp
  edge_se3_pointxyz_depth.h
  types_slam3d.cpp
  types_slam3d.h
  g2o_deprecated_types_slam3d_api.h
)

set_target_properties(deprecated_types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}deprecated_types_slam3d)
if (APPLE)
  set_target_properties(deprecated_types_slam3d PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()
target_link_libraries(deprecated_types_slam3d ${OPENGL_gl_LIBRARY} core types_slam3d)

install(TARGETS deprecated_types_slam3d
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
  )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/deprecated/slam3d)
