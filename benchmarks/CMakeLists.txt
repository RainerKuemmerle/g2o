add_executable(benchmark_jacobian_timing jacobian_timing_tests.cpp)

target_include_directories(benchmark_jacobian_timing PUBLIC
  "$<BUILD_INTERFACE:${g2o_SOURCE_DIR};${g2o_BINARY_DIR}>"
)

target_link_libraries(benchmark_jacobian_timing benchmark::benchmark Eigen3::Eigen)
