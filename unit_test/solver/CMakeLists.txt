add_executable(unittest_solver
  sparse_system_helper.cpp sparse_system_helper.h
  allocate_algorithm_test.cpp
  linear_solver_test.cpp
)

# setting up linking of the test based on the available solvers
set(SOLVER_LIBRARIES solver_eigen solver_dense solver_pcg solver_structure_only)
if(G2O_BUILD_SLAM2D_TYPES)
  list(APPEND SOLVER_LIBRARIES solver_slam2d_linear)
endif()
if(CHOLMOD_FOUND)
  list(APPEND SOLVER_LIBRARIES solver_cholmod)
endif()
if(CSPARSE_FOUND)
  list(APPEND SOLVER_LIBRARIES solver_csparse)
endif()

target_link_libraries(unittest_solver ${SOLVER_LIBRARIES})
create_test(unittest_solver)
