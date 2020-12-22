add_executable(ba_anchored_inverse_depth_demo
  ba_anchored_inverse_depth_demo.cpp
)

target_link_libraries(ba_anchored_inverse_depth_demo core types_sba)
if(CHOLMOD_FOUND)
  target_link_libraries(ba_anchored_inverse_depth_demo solver_cholmod)
else()
  target_link_libraries(ba_anchored_inverse_depth_demo solver_eigen)
endif()

