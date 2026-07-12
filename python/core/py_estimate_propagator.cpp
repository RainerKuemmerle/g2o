#include "py_estimate_propagator.h"

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2opy.h"

namespace g2o {

class PyEstimatePropagatorCostBase : public EstimatePropagatorCostBase {
 public:
  NB_TRAMPOLINE(EstimatePropagatorCostBase, 2);

  // Trampoline (need one for each virtual function)
  double operator()(OptimizableGraph::Edge* edge,
                    const OptimizableGraph::VertexSet& from,
                    OptimizableGraph::Vertex* to) const override {
    NB_OVERRIDE_PURE_NAME("__call__", operator(), edge, from, to);
  }

  // Trampoline (need one for each virtual function)
  [[nodiscard]] std::string_view name() const override {
    NB_OVERRIDE_PURE(name, );
  }
};

void delcareEstimatePropagator(py::module_& m) {
  py::class_<EstimatePropagatorCostBase,
             PyEstimatePropagatorCostBase /* <--- trampoline*/>(
      m, "EstimatePropagatorCostBase")
      .def(py::init<>())
      .def("__call__", &EstimatePropagatorCostBase::operator(), "edge"_a,
           "from"_a, "to"_a, py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           py::keep_alive<1, 4>())  // (OptimizableGraph::Edge* edge, const
      // OptimizableGraph::VertexSet& from,
      // OptimizableGraph::Vertex* to_) -> double
      .def("name", &EstimatePropagatorCostBase::name);

  py::class_<EstimatePropagatorCost, EstimatePropagatorCostBase>(
      m, "EstimatePropagatorCost")
      .def(py::init<SparseOptimizer*>(), "graph"_a, py::keep_alive<1, 2>())
      .def("__call__", &EstimatePropagatorCost::operator(), "edge"_a, "from"_a,
           "to"_a, py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           py::keep_alive<1, 4>())  // (OptimizableGraph::Edge* edge, const
                                    // OptimizableGraph::VertexSet& from,
                                    // OptimizableGraph::Vertex* to_) -> double
      .def("name", &EstimatePropagatorCost::name);

  py::class_<EstimatePropagatorCostOdometry, EstimatePropagatorCostBase>(
      m, "EstimatePropagatorCostOdometry")
      .def(py::init<SparseOptimizer*>(), "graph"_a, py::keep_alive<1, 2>())
      .def("__call__", &EstimatePropagatorCostOdometry::operator(), "edge"_a,
           "from"_a, "to"_a, py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           py::keep_alive<1, 4>())  // (OptimizableGraph::Edge* edge, const
                                    // OptimizableGraph::VertexSet& from,
                                    // OptimizableGraph::Vertex* to_) -> double
      .def("name", &EstimatePropagatorCostOdometry::name);

  py::class_<EstimatePropagator> cls(m, "EstimatePropagator");
  py::class_<EstimatePropagator::PropagateAction>(cls,
                                                  "EstimatePropagateAction")
      .def(py::init<>())
      .def("__call__", &EstimatePropagator::PropagateAction::operator(),
           py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
           py::keep_alive<1, 4>())  // (OptimizableGraph::Edge* edge, const
                                    // OptimizableGraph::VertexSet& from,
                                    // OptimizableGraph::Vertex* to_) -> double
      ;

  py::class_<EstimatePropagator::AdjacencyMapEntry>(
      cls, "EstimatePropagatorAdjacencyMapEntry")
      .def(py::init<>())
      .def("reset", &EstimatePropagator::AdjacencyMapEntry::reset)
      .def("child", &EstimatePropagator::AdjacencyMapEntry::child)
      .def("parent", &EstimatePropagator::AdjacencyMapEntry::parent)
      .def("edge", &EstimatePropagator::AdjacencyMapEntry::edge)
      .def("distance", &EstimatePropagator::AdjacencyMapEntry::distance)
      .def("frontier_level",
           &EstimatePropagator::AdjacencyMapEntry::frontierLevel);

  py::class_<EstimatePropagator::VertexIDHashFunction>(
      cls, "EstimatePropagatorVertexIDHashFunction")
      .def("__call__", &EstimatePropagator::VertexIDHashFunction::operator(),
           "v"_a)  // (const VertexPtr v) -> size_t
      ;

  cls.def(py::init<OptimizableGraph*>(), "g"_a, py::keep_alive<1, 2>());
  cls.def("visited",
          &EstimatePropagator::visited);  // -> OptimizableGraph::VertexSet&
  cls.def("adjacency_map",
          &EstimatePropagator::adjacencyMap);    // -> AdjacencyMap&
  cls.def("graph", &EstimatePropagator::graph);  // -> OptimizableGraph*

  cls.def("propagate",
          static_cast<void (EstimatePropagator::*)(
              const std::shared_ptr<OptimizableGraph::Vertex>&,
              const EstimatePropagatorCostBase&,
              const EstimatePropagator::PropagateAction&, double, double)>(
              &EstimatePropagator::propagate),
          "v"_a, "cost"_a, "action"_a,
          "maxDistance"_a = std::numeric_limits<double>::max(),
          "maxEdgeCost"_a = std::numeric_limits<double>::max(),
          py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
          py::keep_alive<1, 4>());

  cls.def("propagate",
          static_cast<void (EstimatePropagator::*)(
              OptimizableGraph::VertexSet&, const EstimatePropagatorCostBase&,
              const EstimatePropagator::PropagateAction&, double, double)>(
              &EstimatePropagator::propagate),
          "vset"_a, "cost"_a, "action"_a,
          "maxDistance"_a = std::numeric_limits<double>::max(),
          "maxEdgeCost"_a = std::numeric_limits<double>::max(),
          py::keep_alive<1, 2>(), py::keep_alive<1, 3>(),
          py::keep_alive<1, 4>());
}

}  // namespace g2o
