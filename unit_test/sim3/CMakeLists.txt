add_executable(unittest_sim3
  jacobians_sim3.cpp
)
target_link_libraries(unittest_sim3 types_sim3)
create_test(unittest_sim3)
