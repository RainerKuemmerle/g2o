add_executable(unittest_general
  clear_and_redo.cpp
)
target_link_libraries(unittest_general types_slam3d)
target_link_libraries(unittest_general gtest gtest_main)
