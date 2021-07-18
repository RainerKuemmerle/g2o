add_executable(unittest_general
  allocate_optimizer.cpp allocate_optimizer.h
  auto_diff.cpp
  graph_operations.cpp
  clear_and_redo.cpp
  base_fixed_sized_edge.cpp
  robust_kernel_tests.cpp
  sparse_block_matrix.cpp
)
target_link_libraries(unittest_general types_slam3d types_slam2d)
create_test(unittest_general)
