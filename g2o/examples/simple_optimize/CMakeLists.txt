add_executable(simple_optimize
  simple_optimize.cpp
)

set_target_properties(simple_optimize PROPERTIES OUTPUT_NAME simple_optimize)

target_link_libraries(simple_optimize PRIVATE core solver_eigen)
target_link_libraries(simple_optimize PRIVATE types_slam2d)
target_link_libraries(simple_optimize PRIVATE types_slam3d)
target_link_libraries(simple_optimize PRIVATE CLI11::CLI11)
