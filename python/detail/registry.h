#pragma once

#include <pybind11/detail/typeid.h>
#include <pybind11/pybind11.h>

#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2opy.h"
#include "python/trampoline/py_edge_trampoline.h"

namespace g2o::detail {

class Registry {
 public:
  explicit Registry(py::module& m) : mod_(m) {}

  template <typename EdgeType>
  void registerEdgeFixed(const char* name) {
    if constexpr (std::tuple_size_v<typename EdgeType::VertexTypeTuple> == 1) {
      this->registerUnaryEdge<EdgeType::kDimension,
                              typename EdgeType::Measurement,
                              typename EdgeType::VertexXiType>();
      py::class_<
          EdgeType,
          BaseUnaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                        typename EdgeType::VertexXiType>,
          PyEdgeTrampoline<EdgeType>, std::shared_ptr<EdgeType>>(mod_, name)
          .def(py::init<>());
    } else if constexpr (std::tuple_size_v<
                             typename EdgeType::VertexTypeTuple> == 2) {
      this->registerBinaryEdge<
          EdgeType::kDimension, typename EdgeType::Measurement,
          typename EdgeType::VertexXiType, typename EdgeType::VertexXjType>();
      py::class_<
          EdgeType,
          BaseBinaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                         typename EdgeType::VertexXiType,
                         typename EdgeType::VertexXjType>,
          PyEdgeTrampoline<EdgeType>, std::shared_ptr<EdgeType>>(mod_, name)
          .def(py::init<>());
    } else if constexpr (std::tuple_size_v<
                             typename EdgeType::VertexTypeTuple> == 3) {
      this->registerNEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                          typename EdgeType::template NthType<0>,
                          typename EdgeType::template NthType<1>,
                          typename EdgeType::template NthType<2>>();
      py::class_<EdgeType,
                 BaseFixedSizedEdge<EdgeType::kDimension,
                                    typename EdgeType::Measurement,
                                    typename EdgeType::template NthType<0>,
                                    typename EdgeType::template NthType<1>,
                                    typename EdgeType::template NthType<2>>,
                 PyEdgeTrampoline<EdgeType>, std::shared_ptr<EdgeType>>(mod_,
                                                                        name)
          .def(py::init<>());
    } else {
      static_assert(false, "Not implemented size of fixed edge");
    }
  }

  template <typename VertexType>
  void registerVertex(const char* name) {
    this->registerBaseVertex<VertexType::kDimension,
                             typename VertexType::EstimateType>();

    py::class_<
        VertexType,
        BaseVertex<VertexType::kDimension, typename VertexType::EstimateType>,
        std::shared_ptr<VertexType>>(mod_, name)
        .def(py::init<>());
  }

  py::module& mod() { return mod_; }

 protected:
  py::module& mod_;

  std::unordered_set<std::string> registered_edges_;
  std::unordered_set<std::string> registered_vertices_;

  template <int D, typename T>
  void registerBaseVertex() {
    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string estimate_str = pybind11::type_id<T>();

    std::stringstream suffix;
    suffix << dim_str << '_' << estimate_str;
    const std::string base_vertex_name = "BaseVertex" + suffix.str();

    if (registered_vertices_.count(base_vertex_name) > 0) {
      return;
    }
    registered_vertices_.insert(base_vertex_name);

    using CLS = BaseVertex<D, T>;
    using BVector = typename CLS::BVector;

    py::class_<CLS, OptimizableGraph::Vertex, std::shared_ptr<CLS>>(
        mod_, base_vertex_name.c_str())

        //.def(py::init<>())
        //.def_readonly_static("dimension", &BaseVertex<D, T>::Dimension)   //
        // lead to undefined
        // symbol error
        .def("hessian", [](const CLS& v) { return MatrixX(v.hessianMap()); })
        //-> double* .def("map_hessian_memory", &CLS::mapHessianMemory) //
        // double*
        //-> void .def("copy_b", &CLS::copyB) // double* -> void
        .def("clear_quadratic_form", &CLS::clearQuadraticForm)
        .def("solve_direct", &CLS::solveDirect)
        .def("b", static_cast<BVector& (CLS::*)()>(&CLS::b),
             py::return_value_policy::reference)
        //.def("A", (HessianBlockType& (CLS::*) ()) &CLS::A,
        //        py::return_value_policy::reference)

        .def("push", &CLS::push)
        .def("pop", &CLS::pop)
        .def("discard_top", &CLS::discardTop)
        .def("stack_size", &CLS::stackSize)  // -> int

        .def("estimate", &CLS::estimate,
             py::return_value_policy::reference)         // -> T&
        .def("set_estimate", &CLS::setEstimate, "et"_a)  // T& -> void
        ;
  }

  template <int D, typename E, typename VertexXi>
  void registerUnaryEdge() {
    using CLS = BaseUnaryEdge<D, E, VertexXi>;

    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string measurement_str = pybind11::type_id<E>();
    const std::string vi_str = pybind11::type_id<VertexXi>();

    std::stringstream suffix;
    suffix << dim_str << '_' << measurement_str << '_' << vi_str;
    const std::string binary_edge_name = "BaseUnaryEdge" + suffix.str();

    this->registerBaseFixedSizedEdge<D, E, VertexXi>(suffix.str());

    if (registered_edges_.count(binary_edge_name) > 0) {
      return;
    }
    registered_edges_.insert(binary_edge_name);

    py::class_<CLS, BaseFixedSizedEdge<D, E, VertexXi>, BaseEdge<D, E>,
               std::shared_ptr<CLS>>(mod_, binary_edge_name.c_str())
        .def_property(
            "jacobian_oplus_xi",
            [](CLS& cls) { return cls.template jacobianOplusXn<0>(); },
            [](CLS& cls, const MatrixX& m) {
              cls.template jacobianOplusXn<0>() = m;
            })  // -> JacobianXiOplusType&
        ;
  }

  template <int D, typename E, typename VertexXi, typename VertexXj>
  void registerBinaryEdge() {
    using CLS = BaseBinaryEdge<D, E, VertexXi, VertexXj>;

    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string measurement_str = pybind11::type_id<E>();
    const std::string vi_str = pybind11::type_id<VertexXi>();
    const std::string vj_str = pybind11::type_id<VertexXj>();

    std::stringstream suffix;
    suffix << dim_str << '_' << measurement_str << '_' << vi_str << '_'
           << vj_str;
    const std::string binary_edge_name = "BaseBinaryEdge_" + suffix.str();

    this->registerBaseFixedSizedEdge<D, E, VertexXi, VertexXj>(suffix.str());

    if (registered_edges_.count(binary_edge_name) > 0) {
      return;
    }
    registered_edges_.insert(binary_edge_name);

    pybind11::class_<CLS, BaseFixedSizedEdge<D, E, VertexXi, VertexXj>,
                     BaseEdge<D, E>, std::shared_ptr<CLS>>(
        mod_, binary_edge_name.c_str())
        .def_property(
            "jacobian_oplus_xi",
            [](CLS& cls) { return cls.template jacobianOplusXn<0>(); },
            [](CLS& cls, const MatrixX& m) {
              cls.template jacobianOplusXn<0>() = m;
            })  // -> JacobianXiOplusType&
        .def_property(
            "jacobian_oplus_xj",
            [](const CLS& cls) { return cls.template jacobianOplusXn<1>(); },
            [](CLS& cls, const MatrixX& m) {
              cls.template jacobianOplusXn<0>() = m;
            })  //-> JacobianXjOplusType&;
        ;
  }

  template <int D, typename E, typename VertexXi, typename VertexXj,
            typename VertexXk>
  void registerNEdge() {
    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string measurement_str = pybind11::type_id<E>();
    const std::string vi_str = pybind11::type_id<VertexXi>();
    const std::string vj_str = pybind11::type_id<VertexXj>();
    const std::string vk_str = pybind11::type_id<VertexXk>();

    std::stringstream suffix;
    suffix << dim_str << '_' << measurement_str << '_' << vi_str << '_'
           << vj_str << '_' << vk_str;

    this->registerBaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>(
        suffix.str());
  }

  template <int D, typename E, typename... VertexTypes>
  void registerBaseFixedSizedEdge(const std::string& suffix) {
    using CLS = BaseFixedSizedEdge<D, E, VertexTypes...>;

    this->registerBaseEdge<D, E>();

    const std::string base_fixed_size_edge_name = "BaseFixedSizedEdge" + suffix;
    if (registered_edges_.count(base_fixed_size_edge_name) > 0) {
      return;
    }
    registered_edges_.insert(base_fixed_size_edge_name);

    py::class_<CLS, BaseEdge<D, E>, std::shared_ptr<CLS>>(
        mod_, base_fixed_size_edge_name.c_str())
        // abstract class type ..."
        // TODO(Rainer): Fix binding of create_vertex
        //  .def("create_vertex", &CLS::createVertex,
        //       "i"_a)  // -> OptimizableGraph::Vertex*
        .def("resize", &CLS::resize)
        .def("all_vertices_fixed", &CLS::allVerticesFixed)
        .def("compute_error", &CLS::computeError)
        .def("linearize_oplus", static_cast<void (CLS::*)(JacobianWorkspace&)>(
                                    &CLS::linearizeOplus))
        .def("linearize_oplus",
             static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
        .def("set_measurement_data", &CLS::setMeasurementData)
        .def("get_measurement_data", &CLS::getMeasurementData)
        .def("measurement_dimension", &CLS::measurementDimension)
        .def("set_measurement_from_state", &CLS::setMeasurementFromState)
        .def("initial_estimate_possible", &CLS::initialEstimatePossible)
        .def("initial_estimate", &CLS::initialEstimate)
        .def("construct_quadratic_form", &CLS::constructQuadraticForm)
        .def("map_hessian_memory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
             "row_mayor"_a)
        .def("jacobian", &CLS::jacobian, "vertex_index"_a)  // int -> Matrix
        .def("set_jacobian", &CLS::setJacobian, "vertex_index"_a,
             "jacobian"_a)  // int, Matrix ->
        ;
  }

  template <int D, typename E>
  void registerBaseEdge() {
    using CLS = BaseEdge<D, E>;

    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string measurement_str = pybind11::type_id<E>();

    std::stringstream base_edge_name;
    base_edge_name << "BaseEdge_" << dim_str << '_' << measurement_str;
    if (registered_edges_.count(base_edge_name.str()) > 0) {
      return;
    }
    registered_edges_.insert(base_edge_name.str());

    using ErrorVector = typename CLS::ErrorVector;
    using InformationType = typename CLS::InformationType;

    py::class_<CLS, OptimizableGraph::Edge, std::shared_ptr<CLS>>(
        mod_, base_edge_name.str().c_str())
        .def("chi2", &CLS::chi2)
        .def_property(
            "error", [](const CLS& cls) { return cls.error(); },
            [](CLS& cls, ErrorVector& v) { cls.error() = v; })
        .def("information", static_cast<InformationType& (CLS::*)()>(
                                &CLS::information))  // -> InformationType
        .def(
            "set_information",
            [](CLS& edge, const MatrixX& info) { edge.setInformation(info); },
            "information"_a)  // InformationType ->

        .def("measurement", &CLS::measurement)                // -> E
        .def("set_measurement", &CLS::setMeasurement, "m"_a)  // E ->

        .def("rank", &CLS::rank)  // -> int
        .def("initial_estimate",
             &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                     // OptimizableGraph::Vertex*) ->
        ;
  }
};
}  // namespace g2o::detail
