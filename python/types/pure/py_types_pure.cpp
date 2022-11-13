#include "py_types_pure.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <pybind11/cast.h>
#include <pybind11/pytypes.h>

#include <stdexcept>

#include "g2o/config.h"
#include "g2o/core/base_dynamic_vertex.h"
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/eigen_types.h"

namespace g2o {

class VectorXVertex : public BaseDynamicVertex<VectorX> {
 public:
  VectorXVertex() = default;

  bool read(std::istream& is) override {
    int dimension;
    is >> dimension;
    if (!is.good()) {
      return false;
    }
    setDimension(dimension);
    return g2o::internal::readVector(is, estimate_);
  }

  bool write(std::ostream& os) const override {
    os << estimate_.size() << " ";
    return g2o::internal::writeVector(os, estimate_);
  }

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

  void setToOriginImpl() override { estimate_.setZero(); }

  // oplusImpl for python consuming a vector
  virtual void oplus_impl(const VectorX& v) = 0;
  void oplusImpl(const VectorX::MapType& update) override {
    // TODO(goki): Can we use the MapType directly
    oplus_impl(update);
  }
};

class PyVectorXVertex : public VectorXVertex {
 public:
  PyVectorXVertex() = default;

  /* Trampoline for oplusImpl */
  void oplus_impl(const VectorX& v) override {
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

  void computeError() override { error_ = compute_error(); }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

  // python trampoline
  virtual VectorX compute_error() = 0;
};

class PyVariableVectorXEdge : public VariableVectorXEdge {
 public:
  PyVariableVectorXEdge() = default;

  // Trampoline for compute_error
  VectorX compute_error() override {
    PYBIND11_OVERRIDE_PURE(
        VectorX,             /* Return type */
        VariableVectorXEdge, /* Parent class */
        compute_error /* Name of function in C++ (must match Python name) */
    );
  }
};

void declareTypesPure(py::module& m) {
  py::class_<VectorXVertex, PyVectorXVertex,
             BaseVertex<Eigen::Dynamic, VectorX>,
             std::shared_ptr<VectorXVertex>>(m, "VectorXVertex")
      .def(py::init<>())
      .def("estimate", &VectorXVertex::estimate,
           py::return_value_policy::reference)                   // -> T&
      .def("set_estimate", &VectorXVertex::setEstimate, "et"_a)  // T& -> void;
      .def("dimension", &VectorXVertex::dimension)
      .def("set_dimension", &VectorXVertex::setDimension)
      .def("oplus_impl",
           &VectorXVertex::oplus_impl)  // -> void, to be implemented in python
      ;

  py::class_<VariableVectorXEdge, PyVariableVectorXEdge,
             BaseVariableSizedEdge<Eigen::Dynamic, VectorX>,
             std::shared_ptr<VariableVectorXEdge>>(m, "VariableVectorXEdge")
      .def(py::init<>())
      .def("compute_error",
           &VariableVectorXEdge::compute_error)  // -> vector, to be implemented
                                                 // in python
      .def("set_measurement", &VariableVectorXEdge::setMeasurement)
      .def("set_dimension", &VariableVectorXEdge::setDimension<-1>)  // int ->
      // .def("set_measurement_data", &EdgeSE2::setMeasurementData)
      // .def("get_measurement_data", &EdgeSE2::getMeasurementData)
      // .def("measurement_dimension", &EdgeSE2::measurementDimension)
      // .def("set_measurement_from_state", &EdgeSE2::setMeasurementFromState)
      // .def("initial_estimate_possible", &EdgeSE2::initialEstimatePossible)
      // .def("initial_estimate", &EdgeSE2::initialEstimate)
      // .def("linearize_oplus", &EdgeSE2::linearizeOplus)
      ;
}

}  // namespace g2o
