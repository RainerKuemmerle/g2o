find_package(pybind11 REQUIRED)

pybind11_add_module(g2opy g2opy.cpp)
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
