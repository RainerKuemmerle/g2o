add_executable(unittest_slam3d
  jacobians_slam3d.cpp
  mappings_slam3d.cpp
  orthogonal_matrix.cpp
  optimization_slam3d.cpp
)
target_link_libraries(unittest_slam3d types_slam3d)
create_test(unittest_slam3d)
