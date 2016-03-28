add_executable(unittest_general
  ../test_helper/test_main.cpp
  graph_operations.cpp
  clear_and_redo.cpp
)
target_link_libraries(unittest_general types_slam3d types_slam2d)
target_link_libraries(unittest_general ${GTEST_LIBRARIES})
