add_executable(unittest_slam2d
  mappings_se2.cpp
  jacobians_slam2d.cpp
  io_slam2d.cpp
)
target_link_libraries(unittest_slam2d types_slam2d)
create_test(unittest_slam2d)
