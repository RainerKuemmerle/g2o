find_package(benchmark)

if(${benchmark_FOUND})

  add_executable(jacobian_timing_tests jacobian_timing_tests2.cpp)
  target_link_libraries(jacobian_timing_tests benchmark::benchmark Eigen3::Eigen)
  
endif()