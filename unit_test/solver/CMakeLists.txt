add_executable(unittest_solver
  sparse_system_helper.cpp sparse_system_helper.h
  linear_eigen.cpp
)
target_link_libraries(unittest_solver solver_eigen)
create_test(unittest_solver)
