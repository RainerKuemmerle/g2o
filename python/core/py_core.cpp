#include "py_core.h"

#include "base_binary_edge.h"
#include "base_edge.h"
#include "base_multi_edge.h"
#include "base_unary_edge.h"
#include "base_vertex.h"
#include "batch_stats.h"
#include "block_solver.h"
#include "eigen_types.h"
#include "estimate_propagator.h"
#include "hyper_dijkstra.h"
#include "hyper_graph.h"
#include "hyper_graph_action.h"
#include "jacobian_workspace.h"
#include "linear_solver.h"
#include "optimizable_graph.h"
#include "optimization_algorithm.h"
#include "parameter.h"
#include "robust_kernel.h"
#include "solver.h"
#include "sparse_block_matrix.h"
#include "sparse_optimizer.h"
#include "sparse_optimizer_terminate_action.h"

namespace g2o {

void declareCore(pybind11::module& m) {
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
  declareBaseMultiEdge(m);

  declareRobustKernel(m);
  declareSolver(m);
  declareLinearSolver(m);
  declareBlockSolver(m);

  declareOptimizationAlgorithm(m);
  delcareSparseOptimizerTerminateAction(m);
}

}  // namespace g2o
