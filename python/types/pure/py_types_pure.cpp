#include "py_types_pure.h"

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include "detail/registry.h"
#include "g2o/core/base_dynamic_vertex.h"
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/eigen_types.h"

namespace g2o {

class VectorXVertex : public BaseDynamicVertex<VectorX> {
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
  void oplusImpl(const VectorX::MapType& update) override {
    // TODO(goki): Can we use the MapType directly
    oplus_impl(update);
  }
};

class PyVectorXVertex : public VectorXVertex {
 public:
  PyVectorXVertex() = default;

  /* Trampoline for oplusImpl */
  void oplus_impl(const Eigen::Ref<VectorX>& v) override {
    PYBIND11_OVERRIDE_PURE(
        void,          /* Return type */
        VectorXVertex, /* Parent class */
        oplus_impl,    /* Name of function in C++ (must match Python name) */
        v              /* Argument(s) */
    );
  }
};

class VariableVectorXEdge
    : public BaseVariableSizedEdge<Eigen::Dynamic, VectorX> {
 public:
  VariableVectorXEdge() { resize(0); }
};

class PyVariableVectorXEdge : public VariableVectorXEdge {
 public:
  PyVariableVectorXEdge() = default;

  // Trampoline for compute_error
  void computeError() override {
    PYBIND11_OVERRIDE_PURE_NAME(
        void,                /* Return type */
        VariableVectorXEdge, /* Parent class */
        "compute_error",     /* Name of function in Python */
        computeError,        /* function in C++ */
    );
  }

  void linearizeOplus() override {
    PYBIND11_OVERRIDE_NAME(void,                /* Return type */
                           VariableVectorXEdge, /* Parent class */
                           "linearize_oplus",   /* Name of function in Python */
                           linearizeOplus,      /* function in C++ */
    );
  }
};

void declareTypesPure(detail::Registry& registry) {
  registry.registerBaseVertex<Eigen::Dynamic, VectorX>();

  py::class_<VectorXVertex, PyVectorXVertex,
             BaseVertex<Eigen::Dynamic, VectorX>,
             std::shared_ptr<VectorXVertex>>(registry.mod(), "VectorXVertex")
      .def(py::init<>())
      .def("oplus_impl",
           &VectorXVertex::oplus_impl)  // -> void, to be implemented in python
      ;

  registry.registerVariableEdge<VariableVectorXEdge, PyVariableVectorXEdge>(
      "VariableVectorXEdge");
}

}  // namespace g2o
