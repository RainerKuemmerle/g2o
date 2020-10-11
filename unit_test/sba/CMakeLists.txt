add_executable(unittest_sba
  io_sba.cpp
  io_six_dof_expmap.cpp
)
target_link_libraries(unittest_sba types_sba)
create_test(unittest_sba)
