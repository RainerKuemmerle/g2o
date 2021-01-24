find_package(pybind11 REQUIRED)

pybind11_add_module(g2opy
    # core
    core/base_edge.cpp
core/base_multi_edge.cpp
core/base_vertex.cpp
core/batch_stats.cpp
core/block_solver.cpp
core/py_core.cpp
core/eigen_types.cpp
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
