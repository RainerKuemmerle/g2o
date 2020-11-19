cmake_minimum_required(VERSION 2.6)

add_subdirectory(stuff)
add_subdirectory(core)

if(G2O_BUILD_APPS)
  add_subdirectory(apps)
endif(G2O_BUILD_APPS)

# Pre-canned types
add_subdirectory(types)

# Solvers
add_subdirectory(solvers)

# Examples
if(G2O_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(G2O_BUILD_EXAMPLES)

# Benchmarks
if(G2O_BUILD_BENCHMARKS)
  find_package(benchmark)
  if(${benchmark_FOUND})
    add_subdirectory(benchmarks)
  else()
    message(WARNING G2O_BUILD_BENCHMARKS was sent to true, but the benchmark library cannot be found)
  endif()
endif()

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/playground")
  # Playground
  option (G2O_BUILD_PLAYGROUND "Build g2o playground" ON)
  if (G2O_BUILD_PLAYGROUND)
    message(STATUS "building playground")
    add_subdirectory(playground)
  endif()
endif()
