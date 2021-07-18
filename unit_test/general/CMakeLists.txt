add_executable(unittest_general
  auto_diff.cpp
  graph_operations.cpp
  clear_and_redo.cpp
  base_fixed_sized_edge.cpp
  robust_kernel_tests.cpp
  sparse_block_matrix.cpp
)
target_link_libraries(unittest_general unittest_helper types_slam3d types_slam2d)
create_test(unittest_general)
