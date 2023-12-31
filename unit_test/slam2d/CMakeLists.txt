add_executable(unittest_slam2d
  mappings_se2.cpp
  jacobians_slam2d.cpp
  slam2d_io.cpp
  solve_direct_slam2d.cpp
)
target_link_libraries(unittest_slam2d unittest_helper types_slam2d)
create_test(unittest_slam2d)
