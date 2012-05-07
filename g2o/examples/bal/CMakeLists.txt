ADD_EXECUTABLE(bal_example
  bal_example.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(bal_example core solver_cholmod types_slam3d)
