#include "py_types_pure.h"

#include "detail/registry.h"
#include "g2o/core/base_dynamic_vertex.h"
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2opy.h"

namespace g2o {

namespace {

class VectorXVertex : public BaseDynamicVertex<VectorXVertex, VectorX> {
 public:
  VectorXVertex() = default;

  bool setDimensionImpl(int newDimension) override {
    const int oldDimension = dimension();
    if (oldDimension == Eigen::Dynamic) {
      estimate_.resize(newDimension);
      estimate_.setZero();
      return true;
    }
    estimate_.conservativeResize(newDimension);
    if (oldDimension < newDimension)
      estimate_.tail(newDimension - oldDimension).setZero();
    return true;
  }

  // oplusImpl for python consuming a vector
  virtual void oplus_impl(const Eigen::Ref<VectorX>& v) = 0;
  void oplusImpl(const VectorX::MapType& update) {
    // TODO(goki): Can we use the MapType directly
    oplus_impl(update);
  }
};

class PyVectorXVertex : public VectorXVertex {
 public:
  NB_TRAMPOLINE(VectorXVertex, 1);
  PyVectorXVertex() = default;

  /* Trampoline for oplusImpl */
  void oplus_impl(const Eigen::Ref<VectorX>& v) override {
    NB_OVERRIDE_PURE(oplus_impl, v);
  }
};

class VariableVectorXEdge
    : public BaseVariableSizedEdge<Eigen::Dynamic, VectorX> {
 public:
  VariableVectorXEdge() { resize(0); }
};

class PyVariableVectorXEdge : public VariableVectorXEdge {
 public:
  NB_TRAMPOLINE(VariableVectorXEdge, 2);
  PyVariableVectorXEdge() = default;

  // Trampoline for compute_error
  void computeError() override {
    NB_OVERRIDE_PURE_NAME("compute_error", computeError, );
  }

  void linearizeOplus() override {
    NB_OVERRIDE_NAME("linearize_oplus", linearizeOplus, );
  }
};

}  // namespace

void declareTypesPure(detail::Registry& registry) {
  py::class_<VectorXVertex, PyVectorXVertex, OptimizableGraph::Vertex>(
      registry.mod(), "VectorXVertex")
      .def(py::init<>())
      .def("hessian",
           [](const VectorXVertex& v) { return MatrixX(v.hessianMap()); })
      .def("b", static_cast<VectorX& (VectorXVertex::*)()>(&VectorXVertex::b))
      .def("estimate", &VectorXVertex::estimate)
      .def("set_estimate", &VectorXVertex::setEstimate, "et"_a,
           py::keep_alive<1, 2>())
      .def("oplus_impl",
           &VectorXVertex::oplus_impl)  // -> void, to be implemented in python
      ;

  registry.registerVariableEdge<VariableVectorXEdge, PyVariableVectorXEdge>(
      "VariableVectorXEdge");
}

}  // namespace g2o
