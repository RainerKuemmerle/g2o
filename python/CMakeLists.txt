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
    types/slam3d/py_types_slam3d.cpp
    # module main
    g2opy.cpp)

target_link_libraries(g2opy PRIVATE
    core
    solver_cholmod
    solver_csparse
    solver_eigen
    solver_dense
    solver_pcg
    solver_slam2d_linear
    solver_structure_only
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
