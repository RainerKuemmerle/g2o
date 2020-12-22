add_executable(gicp_demo
  gicp_demo.cpp
)

add_executable(gicp_sba_demo
  gicp_sba_demo.cpp
)

set_target_properties(gicp_sba_demo PROPERTIES OUTPUT_NAME gicp_sba_demo${EXE_POSTFIX})

target_link_libraries(gicp_demo core types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} solver_eigen)
target_link_libraries(gicp_sba_demo core types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} solver_eigen)
