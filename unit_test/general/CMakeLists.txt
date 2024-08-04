add_executable(unittest_general
  auto_diff.cpp
  batch_statistics.cpp
  graph_io.cpp
  graph_io_dynamic.cpp
  graph_operations.cpp
  clear_and_redo.cpp
  base_fixed_sized_edge.cpp
  robust_kernel_tests.cpp
  sparse_block_matrix.cpp
  type_traits_tests.cpp
  optimization_basics.cpp
)
target_link_libraries(unittest_general unittest_helper)
target_link_libraries(unittest_general types_slam3d types_slam2d types_data)
create_test(unittest_general)
