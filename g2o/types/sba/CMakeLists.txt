add_library(types_sba ${G2O_LIB_TYPE}
  # API
  g2o_types_sba_api.h
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  # SBA vertices
  vertex_intrinsics.cpp vertex_intrinsics.h
  vertex_cam.cpp vertex_cam.h
  # SBA edges
  edge_sba_cam.cpp edge_sba_cam.h
  edge_sba_scale.cpp edge_sba_scale.h
  edge_project_p2mc.cpp edge_project_p2mc.h
  edge_project_p2sc.cpp edge_project_p2sc.h
  # other types
  sbacam.cpp sbacam.h
  # six_dof_expmap parameters
  parameter_cameraparameters.cpp parameter_cameraparameters.h
  # six_dof_expmap vertices
  vertex_se3_expmap.cpp vertex_se3_expmap.h
  # six_dof_expmap edges
  edge_project_xyz2uv.cpp edge_project_xyz2uv.h
  edge_project_xyz2uv.cpp edge_project_xyz2uv.h
  edge_project_xyz2uvu.cpp edge_project_xyz2uvu.h
  edge_project_xyz.cpp edge_project_xyz.h
  edge_project_psi2uv.cpp edge_project_psi2uv.h
  edge_se3_expmap.cpp edge_se3_expmap.h
  edge_project_stereo_xyz.cpp edge_project_stereo_xyz.h
  # TBD
  edge_project_xyz_onlypose.cpp edge_project_xyz_onlypose.h
  edge_project_stereo_xyz_onlypose.cpp edge_project_stereo_xyz_onlypose.h
)

set_target_properties(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)
if (APPLE)
  set_target_properties(types_sba PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_link_libraries(types_sba core types_slam3d)

install(TARGETS types_sba
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/sba)
