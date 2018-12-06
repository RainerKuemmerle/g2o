add_executable(unittest_general
  graph_operations.cpp
  clear_and_redo.cpp
  base_fixed_sized_edge.cpp
)
target_link_libraries(unittest_general types_slam3d types_slam2d)
create_test(unittest_general)
