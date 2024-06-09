add_executable(unittest_simulator
  simulator2D_tests.cpp
  simulator3D_tests.cpp
)
target_link_libraries(unittest_simulator unittest_helper simulator_lib)
create_test(unittest_simulator)
