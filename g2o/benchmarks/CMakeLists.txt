find_package(benchmark)

if(${benchmark_FOUND})

  add_executable(jacobian_timing_tests jacobian_timing_tests.cpp)
  target_link_libraries(jacobian_timing_tests benchmark::benchmark ${G2O_EIGEN3_EIGEN_TARGET})
  
endif()