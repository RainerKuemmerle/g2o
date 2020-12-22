add_executable(ba_demo
  ba_demo.cpp
)

target_link_libraries(ba_demo core types_sba)
if(CHOLMOD_FOUND)
  target_link_libraries(ba_demo solver_cholmod)
else()
  target_link_libraries(ba_demo solver_eigen)
endif()
