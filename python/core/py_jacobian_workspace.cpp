#include "py_jacobian_workspace.h"

#include <g2o/core/jacobian_workspace.h>
#include <g2o/core/optimizable_graph.h>

namespace g2o {

void declareJacobianWorkspace(py::module& m) {
  py::class_<JacobianWorkspace>(m, "JacobianWorkspace")
      .def(py::init<>())
      .def("allocate", &JacobianWorkspace::allocate)
      .def("update_size",
           static_cast<void (JacobianWorkspace::*)(
               const HyperGraph::Edge*, bool)>(&JacobianWorkspace::updateSize),
           "e"_a, "reset"_a = false)
      .def("update_size",
           static_cast<void (JacobianWorkspace::*)(
               const OptimizableGraph&, bool)>(&JacobianWorkspace::updateSize),
           "graph"_a, "reset"_a = false)
      .def("update_size",
           static_cast<void (JacobianWorkspace::*)(int, int, bool)>(
               &JacobianWorkspace::updateSize),
           "num_vertices"_a, "dimension"_a, "reset"_a = false)

      .def("workspace_for_vertex", &JacobianWorkspace::workspaceForVertex,
           "vertex_index"_a)  // int -> double*
      ;
}

}  // end namespace g2o
