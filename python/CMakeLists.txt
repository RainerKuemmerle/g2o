# download pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.10.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(pybind11)

###################################################
# python wrapper library
pybind11_add_module(g2opy
    # core
    core/py_base_edge.cpp
    core/py_base_variable_sized_edge.cpp
    core/py_base_vertex.cpp
    core/py_batch_stats.cpp
    core/py_block_solver.cpp
    core/py_core.cpp
    core/py_eigen_types.cpp
    core/py_estimate_propagator.cpp
    core/py_hyper_dijkstra.cpp
    core/py_hyper_graph.cpp
    core/py_jacobian_workspace.cpp
    core/py_optimizable_graph.cpp
    core/py_optimization_algorithm.cpp
    core/py_robust_kernel.cpp
    core/py_sparse_block_matrix.cpp
    core/py_sparse_optimizer.cpp
    # types
    types/icp/py_types_icp.cpp
    types/pure/py_types_pure.cpp
    types/sclam2d/py_types_sclam2d.cpp
    types/sim3/py_types_seven_dof_expmap.cpp
    types/slam2d/py_types_slam2d.cpp
    types/slam3d/py_types_slam3d.cpp
    types/sba/py_types_sba.cpp
    types/sba/py_types_six_dof_expmap.cpp
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
install(TARGETS g2opy LIBRARY DESTINATION g2opy)
