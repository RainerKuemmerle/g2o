#include "py_core.h"

#include "py_base_binary_edge.h"
#include "py_base_edge.h"
#include "py_base_unary_edge.h"
#include "py_base_variable_sized_edge.h"
#include "py_base_vertex.h"
#include "py_batch_stats.h"
#include "py_block_solver.h"
#include "py_eigen_types.h"
#include "py_estimate_propagator.h"
#include "py_hyper_dijkstra.h"
#include "py_hyper_graph.h"
#include "py_hyper_graph_action.h"
#include "py_jacobian_workspace.h"
#include "py_linear_solver.h"
#include "py_optimizable_graph.h"
#include "py_optimization_algorithm.h"
#include "py_parameter.h"
#include "py_robust_kernel.h"
#include "py_solver.h"
#include "py_sparse_block_matrix.h"
#include "py_sparse_optimizer.h"
#include "py_sparse_optimizer_terminate_action.h"

namespace g2o {

void declareCore(py::module& m) {
  declareHyperGraph(m);
  declareOptimizableGraph(m);
  declareSparseOptimizer(m);

  delcareHyperGraphAction(m);
  delcareHyperDijkstra(m);
  delcareEstimatePropagator(m);
  delcareSparseBlockMatrix(m);

  declareEigenTypes(m);
  declareParameter(m);
  declareG2OBatchStatistics(m);

  declareJacobianWorkspace(m);
  declareBaseVertex(m);
  declareBaseEdge(m);
  declareBaseVariableSizedEdge(m);

  declareRobustKernel(m);
  declareSolver(m);
  declareBlockSolver(m);

  declareOptimizationAlgorithm(m);
  delcareSparseOptimizerTerminateAction(m);
}

}  // namespace g2o
