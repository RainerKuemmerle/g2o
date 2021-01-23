#include <pybind11/pybind11.h>

#include "core/base_binary_edge.h"
#include "core/base_edge.h"
#include "core/base_multi_edge.h"
#include "core/base_unary_edge.h"
#include "core/base_vertex.h"
#include "core/batch_stats.h"
#include "core/block_solver.h"
#include "core/eigen_types.h"
#include "core/estimate_propagator.h"
#include "core/hyper_dijkstra.h"
#include "core/hyper_graph.h"
#include "core/hyper_graph_action.h"
#include "core/jacobian_workspace.h"
#include "core/linear_solver.h"
#include "core/optimizable_graph.h"
#include "core/optimization_algorithm.h"
#include "core/parameter.h"
#include "core/robust_kernel.h"
#include "core/solver.h"
#include "core/sparse_block_matrix.h"
#include "core/sparse_optimizer.h"
#include "core/sparse_optimizer_terminate_action.h"
#include "types/types.h"

namespace py = pybind11;

namespace g2o {

PYBIND11_MODULE(g2opy, m) {
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
  declareBaseUnaryEdge(m);
  declareBaseBinaryEdge(m);
  declareBaseMultiEdge(m);

  declareRobustKernel(m);
  declareSolver(m);
  declareLinearSolver(m);
  declareBlockSolver(m);

  declareOptimizationAlgorithm(m);
  delcareSparseOptimizerTerminateAction(m);

  declareTypes(m);
}

}  // namespace g2o
