add_executable(ba_anchored_inverse_depth_demo
  ba_anchored_inverse_depth_demo.cpp
)

include_directories(${CHOLMOD_INCLUDE_DIR})

target_link_libraries(ba_anchored_inverse_depth_demo core solver_cholmod types_sba)


