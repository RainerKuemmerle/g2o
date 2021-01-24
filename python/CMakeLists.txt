find_package(pybind11 REQUIRED)

pybind11_add_module(g2opy
    # core
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
