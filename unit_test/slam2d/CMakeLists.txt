add_executable(unittest_slam2d
  mappings_se2.cpp
  jacobians_slam2d.cpp
  odom_convert_sclam2d.cpp
)
target_link_libraries(unittest_slam2d types_slam2d types_sclam2d)
create_test(unittest_slam2d)
