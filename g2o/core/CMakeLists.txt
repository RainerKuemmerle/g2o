ADD_LIBRARY(core ${G2O_LIB_TYPE}
dynamic_aligned_buffer.hpp
ownership.h
base_edge.h
base_binary_edge.h          hyper_graph_action.cpp
base_binary_edge.hpp        hyper_graph_action.h
base_multi_edge.h           hyper_graph.cpp
base_multi_edge.hpp         hyper_graph.h
base_unary_edge.h           linear_solver.h
base_unary_edge.hpp         marginal_covariance_cholesky.cpp
base_vertex.h               marginal_covariance_cholesky.h
base_vertex.hpp             matrix_structure.cpp
batch_stats.cpp             matrix_structure.h
batch_stats.h               openmp_mutex.h
block_solver.h              block_solver.hpp            
parameter.cpp               parameter.h                 
cache.cpp                   cache.h
optimizable_graph.cpp       optimizable_graph.h         
solver.cpp                  solver.h
creators.h                  optimization_algorithm_factory.cpp
estimate_propagator.cpp     optimization_algorithm_factory.h
estimate_propagator.h       
factory.cpp                 optimization_algorithm_property.h
factory.h                   sparse_block_matrix.h
sparse_optimizer.cpp  sparse_block_matrix.hpp
sparse_optimizer.h
hyper_dijkstra.cpp hyper_dijkstra.h
parameter_container.cpp     parameter_container.h
optimization_algorithm.cpp optimization_algorithm.h
optimization_algorithm_with_hessian.cpp optimization_algorithm_with_hessian.h
optimization_algorithm_gauss_newton.cpp optimization_algorithm_gauss_newton.h
optimization_algorithm_levenberg.cpp optimization_algorithm_levenberg.h
optimization_algorithm_dogleg.cpp optimization_algorithm_dogleg.h
sparse_optimizer_terminate_action.cpp sparse_optimizer_terminate_action.h
jacobian_workspace.cpp jacobian_workspace.h
robust_kernel.cpp robust_kernel.h
robust_kernel_impl.cpp robust_kernel_impl.h
robust_kernel_factory.cpp robust_kernel_factory.h
g2o_core_api.h
)

SET_TARGET_PROPERTIES(core PROPERTIES OUTPUT_NAME ${LIB_PREFIX}core)

TARGET_LINK_LIBRARIES(core stuff)

IF(G2O_USE_EIGEN3_EIGEN_TARGET)
  TARGET_LINK_LIBRARIES(core stuff Eigen3::Eigen)
  SET(G2O_EIGEN3_INCLUDES_DIR)
ELSE(G2O_USE_EIGEN3_EIGEN_TARGET)
  SET(G2O_EIGEN3_INCLUDES_DIR ${EIGEN3_INCLUDE_DIR})
ENDIF(G2O_USE_EIGEN3_EIGEN_TARGET)
  
SET(include_install_dir ${CMAKE_INSTALL_PREFIX}/include)

INSTALL(TARGETS core
  EXPORT ${targets_export_name}
  RUNTIME DESTINATION ${runtime_destination}
  LIBRARY DESTINATION ${library_destination}
  ARCHIVE DESTINATION ${archive_destination}
  INCLUDES DESTINATION ${includes_destination} ${G2O_EIGEN3_INCLUDES_DIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${includes_destination}/g2o/core)
