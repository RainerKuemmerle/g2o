ADD_EXECUTABLE(ba_anchored_inverse_depth_demo
  ba_anchored_inverse_depth_demo.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(ba_anchored_inverse_depth_demo core solver_cholmod types_sba)


