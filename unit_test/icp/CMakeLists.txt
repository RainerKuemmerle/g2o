add_executable(unittest_icp
  icp_rotation.cpp
)
target_link_libraries(unittest_icp types_icp)
create_test(unittest_icp)
