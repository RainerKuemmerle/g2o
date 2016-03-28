add_executable(unittest_slam3d
  ../test_helper/test_main.cpp
  jacobians_slam3d.cpp
  mappings_slam3d.cpp
  orthogonal_matrix.cpp
)
target_link_libraries(unittest_slam3d types_slam3d)
target_link_libraries(unittest_slam3d ${GTEST_LIBRARIES})
