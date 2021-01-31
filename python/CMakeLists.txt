# download pybind11
configure_file(pybind11/CMakeLists.txt.in ${CMAKE_BINARY_DIR}/pybind11-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/pybind11-download )
if(result)
  message(FATAL_ERROR "CMake step for pybind11 failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/pybind11-download )
if(result)
  message(FATAL_ERROR "Build step for pybind11 failed: ${result}")
endif()

# Add pybind11 directly to our build.
add_subdirectory(${CMAKE_BINARY_DIR}/pybind11-src
                 ${CMAKE_BINARY_DIR}/pybind11-build
                 EXCLUDE_FROM_ALL)

###################################################
# python wrapper library
pybind11_add_module(g2opy
    # # core
    core/py_base_edge.cpp
    core/py_base_variable_sized_edge.cpp
    core/py_base_vertex.cpp
    core/py_batch_stats.cpp
    core/py_block_solver.cpp
    core/py_core.cpp
    core/py_eigen_types.cpp
    # types
    types/slam2d/py_types_slam2d.cpp
    types/slam3d/py_types_slam3d.cpp
    # module main
    g2opy.cpp)

set(SOLVER_LIBRARIES solver_eigen solver_dense solver_pcg solver_slam2d_linear solver_structure_only)
if(CHOLMOD_FOUND)
    list(APPEND SOLVER_LIBRARIES solver_cholmod)
endif()
if(CSPARSE_FOUND)
    list(APPEND SOLVER_LIBRARIES solver_csparse)
endif()
target_link_libraries(g2opy PRIVATE ${SOLVER_LIBRARIES})

target_link_libraries(g2opy PRIVATE
    core
    types_data
    types_icp
    types_sba
    types_sclam2d
    types_sim3
    types_slam2d
    types_slam2d_addons
    types_slam3d
    types_slam3d_addons
)
target_include_directories(g2opy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
