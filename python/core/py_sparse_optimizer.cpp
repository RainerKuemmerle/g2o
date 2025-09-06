#include "py_sparse_optimizer.h"

#include <g2o/core/estimate_propagator.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/core/sparse_optimizer.h>

#include <utility>

namespace g2o {

void declareSparseOptimizer(py::module& m) {
  using CLS = SparseOptimizer;

  py::class_<CLS, OptimizableGraph>(m, "SparseOptimizer")
      // ATTENTION: _solver & _statistics is own by SparseOptimizer and will be
      // deleted in its destructor.
      .def(py::init<>())

      .def("initialize_optimization",
           static_cast<bool (CLS::*)(HyperGraph::EdgeSet&)>(
               &CLS::initializeOptimization),
           "eset"_a)  // virtual
      .def("initialize_optimization",
           static_cast<bool (CLS::*)(HyperGraph::VertexSet&, int)>(
               &CLS::initializeOptimization),
           "vset"_a,
           "level"_a = 0)  // virtual
      .def("initialize_optimization",
           static_cast<bool (CLS::*)(int)>(&CLS::initializeOptimization),
           "level"_a = 0)  // virtual

      .def("update_initialization", &CLS::updateInitialization, "vset"_a,
           "eset"_a)  // virtual, ->bool
      .def("compute_initial_guess",
           static_cast<void (CLS::*)()>(&CLS::computeInitialGuess))  // virtual
      .def("compute_initial_guess",
           static_cast<void (CLS::*)(EstimatePropagatorCostBase&)>(
               &CLS::computeInitialGuess),
           "propagator"_a)  // virtual

      .def("optimize", &CLS::optimize, "iterations"_a, "online"_a = false,
           py::call_guard<py::gil_scoped_release>())  // -> int
      .def(
          "compute_marginals",
          [](CLS& optimizer,
             const std::vector<std::pair<int, int> >& block_indices) {
            auto* spinv = new SparseBlockMatrix<MatrixX>();
            bool result = optimizer.computeMarginals(*spinv, block_indices);
            return std::make_pair(spinv, result);
          },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "compute_marginals",
          [](CLS& optimizer, const OptimizableGraph::Vertex* vertex) {
            auto* spinv = new SparseBlockMatrix<MatrixX>();
            bool result = optimizer.computeMarginals(*spinv, vertex);
            return std::make_pair(spinv, result);
          },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "compute_marginals",
          [](CLS& optimizer,
             const OptimizableGraph::VertexContainer& vertices) {
            auto* spinv = new SparseBlockMatrix<MatrixX>();
            bool result = optimizer.computeMarginals(*spinv, vertices);
            return std::make_pair(spinv, result);
          },
          py::call_guard<py::gil_scoped_release>())

      // The gauge should be fixed() and then the optimization can work (if no
      // additional dof are in the system. The default implementation returns a
      // node with maximum dimension.
      .def("find_gauge", &CLS::findGauge,
           py::return_value_policy::reference)   // virtual, ->
                                                 // OptimizableGraph::Vertex*
      .def("gauge_freedom", &CLS::gaugeFreedom)  // -> bool
      .def("active_chi2", &CLS::activeChi2)      // -> double
      .def("active_robust_chi2", &CLS::activeRobustChi2)  // -> double
      .def("verbose", &CLS::verbose)                      // -> bool
      .def("set_verbose", &CLS::setVerbose,
           "verbose"_a)  // -> void

      .def("set_force_stop_flag", &CLS::setForceStopFlag,
           "flag"_a)                                // -> void
      .def("force_stop_flag", &CLS::forceStopFlag)  // -> bool*
      .def("terminate", &CLS::terminate)            // -> bool

      .def("index_mapping", &CLS::indexMapping)  // -> const VertexContainer&
      .def("active_vertices",
           &CLS::activeVertices)               // -> const VertexContainer&
      .def("active_edges", &CLS::activeEdges)  // -> const EdgeContainer&
      .def("remove_vertex", &CLS::removeVertex, "v"_a)  // virtual, -> bool

      .def("find_active_vertex", &CLS::findActiveVertex,
           "v"_a)  // -> VertexContainer::const_iterator, const
      .def("find_active_edge", &CLS::findActiveEdge,
           "e"_a)  // -> EdgeContainer::const_iterator, const

      .def("algorithm", &CLS::algorithm,
           py::return_value_policy::reference)  // -> const
                                                // OptimizationAlgorithm&, const
      .def("solver", &CLS::solver,
           py::return_value_policy::reference)  // -> OptimizationAlgorithm*
      //.def("set_algorithm", &CLS::setAlgorithm,
      //        "algorithm"_a, py::keep_alive<1, 2>()) // -> void

      .def(
          "set_algorithm",
          [](CLS& optimizer,
             const std::shared_ptr<OptimizationAlgorithm>& algorithm) {
            optimizer.setAlgorithm(algorithm);
          },
          py::keep_alive<1, 2>())
      .def(
          "set_algorithm",
          [](CLS& optimizer,
             const std::shared_ptr<OptimizationAlgorithmWithHessian>&
                 algorithm) { optimizer.setAlgorithm(algorithm); },
          py::keep_alive<1, 2>())
      .def(
          "set_algorithm",
          [](CLS& optimizer,
             const std::shared_ptr<OptimizationAlgorithmGaussNewton>&
                 algorithm) { optimizer.setAlgorithm(algorithm); },
          py::keep_alive<1, 2>())
      .def(
          "set_algorithm",
          [](CLS& optimizer,
             const std::shared_ptr<OptimizationAlgorithmLevenberg>& algorithm) {
            optimizer.setAlgorithm(algorithm);
          },
          py::keep_alive<1, 2>())
      .def(
          "set_algorithm",
          [](CLS& optimizer,
             const std::shared_ptr<OptimizationAlgorithmDogleg>& algorithm) {
            optimizer.setAlgorithm(algorithm);
          },
          py::keep_alive<1, 2>())

      .def("push",
           static_cast<void (CLS::*)(SparseOptimizer::VertexContainer&)>(
               &CLS::push),
           "vlist"_a)
      .def("push",
           static_cast<void (CLS::*)(HyperGraph::VertexSet&)>(&CLS::push),
           "vlist"_a)
      .def("push", static_cast<void (CLS::*)()>(&CLS::push))

      .def("pop",
           static_cast<void (CLS::*)(SparseOptimizer::VertexContainer&)>(
               &CLS::pop),
           "vlist"_a)
      .def("pop", static_cast<void (CLS::*)(HyperGraph::VertexSet&)>(&CLS::pop),
           "vlist"_a)
      .def("pop", static_cast<void (CLS::*)()>(&CLS::pop))

      .def("discard_top",
           static_cast<void (CLS::*)(SparseOptimizer::VertexContainer&)>(
               &CLS::discardTop))  // -> void
      .def("discard_top", static_cast<void (CLS::*)()>(&CLS::discardTop))

      .def("clear", &CLS::clear)  // virtual, -> void
      .def("compute_active_errors",
           &CLS::computeActiveErrors)           // virtual, -> void
      .def("update", &CLS::update, "update"_a)  // -> void

      .def("batch_statistics",
           static_cast<const BatchStatisticsContainer& (CLS::*)() const>(
               &CLS::batchStatistics))
      .def("set_compute_batch_statistics",
           &CLS::setComputeBatchStatistics)  // -> void
      .def("compute_batch_statistics", &CLS::computeBatchStatistics)

      // callbacks
      .def("add_compute_error_action", &CLS::addComputeErrorAction,
           "action"_a)  // -> bool
      .def("remove_compute_error_action", &CLS::removeComputeErrorAction,
           "action"_a)  // -> bool

      ;
}

}  // end namespace g2o
