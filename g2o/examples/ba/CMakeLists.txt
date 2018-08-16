add_executable(ba_demo
  ba_demo.cpp
)

include_directories(${CHOLMOD_INCLUDE_DIR})

target_link_libraries(ba_demo core solver_cholmod types_sba)


