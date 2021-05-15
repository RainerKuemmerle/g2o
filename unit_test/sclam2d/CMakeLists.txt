add_executable(unittest_sclam2d
  io_sclam2d.cpp
  odom_convert_sclam2d.cpp
  sensor_offset.cpp
)
target_link_libraries(unittest_sclam2d unittest_helper types_sclam2d)
create_test(unittest_sclam2d)
