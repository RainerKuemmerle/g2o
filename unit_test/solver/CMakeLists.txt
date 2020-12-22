add_executable(unittest_solver
  sparse_system_helper.cpp sparse_system_helper.h
  linear_solver_test.cpp
)

# setting up linking of the test based on the available solvers
set(SOLVER_LIBRARIES solver_eigen)
if(CHOLMOD_FOUND)
  list(APPEND SOLVER_LIBRARIES solver_cholmod)
endif()
if(CSPARSE_FOUND)
  list(APPEND SOLVER_LIBRARIES solver_csparse)
endif()

target_link_libraries(unittest_solver ${SOLVER_LIBRARIES})
create_test(unittest_solver)
