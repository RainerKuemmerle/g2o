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
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2opy.h"
#include "python/trampoline/py_edge_trampoline.h"

namespace g2o::detail {

class Registry {
 public:
  explicit Registry(py::module& m)
      : mod_(m),
        private_mod_(mod_.def_submodule(
            "private", "Classes private to the implementation")) {}

  template <typename EdgeType, typename Trampoline = PyEdgeTrampoline<EdgeType>>
  auto registerEdgeFixed(const char* name) {
    if constexpr (std::tuple_size_v<typename EdgeType::VertexTypeTuple> == 1) {
      this->registerUnaryEdge<EdgeType::kDimension,
                              typename EdgeType::Measurement,
                              typename EdgeType::VertexXiType>();
      return py::classh<EdgeType,
                        BaseUnaryEdge<EdgeType::kDimension,
                                      typename EdgeType::Measurement,
                                      typename EdgeType::VertexXiType>,
                        Trampoline>(mod_, name)
          .def(py::init<>());
    } else if constexpr (std::tuple_size_v<
                             typename EdgeType::VertexTypeTuple> == 2) {
      this->registerBinaryEdge<
          EdgeType::kDimension, typename EdgeType::Measurement,
          typename EdgeType::VertexXiType, typename EdgeType::VertexXjType>();
      return py::classh<EdgeType,
                        BaseBinaryEdge<EdgeType::kDimension,
                                       typename EdgeType::Measurement,
                                       typename EdgeType::VertexXiType,
                                       typename EdgeType::VertexXjType>,
                        Trampoline>(mod_, name)
          .def(py::init<>());
    } else if constexpr (std::tuple_size_v<
                             typename EdgeType::VertexTypeTuple> == 3) {
      this->registerNEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                          typename EdgeType::template NthType<0>,
                          typename EdgeType::template NthType<1>,
                          typename EdgeType::template NthType<2>>();
      return py::classh<
                 EdgeType,
                 BaseFixedSizedEdge<EdgeType::kDimension,
                                    typename EdgeType::Measurement,
                                    typename EdgeType::template NthType<0>,
                                    typename EdgeType::template NthType<1>,
                                    typename EdgeType::template NthType<2>>,
                 Trampoline>(mod_, name)
          .def(py::init<>());
    } else {
      static_assert(
          std::tuple_size_v<typename EdgeType::VertexTypeTuple> < 0 ||
              std::tuple_size_v<typename EdgeType::VertexTypeTuple> > 3,
          "Not implemented size of fixed edge");
      return 0;
    }
  }

  template <typename EdgeType, typename Trampoline = PyEdgeTrampoline<EdgeType>>
  auto registerVariableEdge(const char* name) {
    this->registerBaseVariableEdge<EdgeType::kDimension,
                                   typename EdgeType::Measurement>();
    return py::classh<EdgeType,
                      BaseVariableSizedEdge<EdgeType::kDimension,
                                            typename EdgeType::Measurement>,
                      Trampoline>(mod_, name)
        .def(py::init<>());
  }

  template <typename VertexType>
  auto registerVertex(const char* name) {
    this->registerBaseVertex<VertexType::kDimension,
                             typename VertexType::EstimateType>();

    return py::classh<VertexType,
                      BaseVertex<VertexType::kDimension,
                                 typename VertexType::EstimateType>>(mod_, name)
        .def(py::init<>());
  }

  py::module& mod() { return mod_; }

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

    py::classh<CLS, OptimizableGraph::Vertex>(private_mod_,
                                              base_vertex_name.c_str())

        //.def(py::init<>())
        //.def_readonly_static("dimension", &BaseVertex<D, T>::Dimension)   //
        // lead to undefined
        // symbol error
        .def("hessian", [](const CLS& v) { return MatrixX(v.hessianMap()); })
        .def("b", static_cast<BVector& (CLS::*)()>(&CLS::b))
        //.def("A", (HessianBlockType& (CLS::*) ()) &CLS::A)
        .def("estimate", &CLS::estimate)  // -> T&
        .def("set_estimate", &CLS::setEstimate, "et"_a,
             py::keep_alive<1, 2>())  // T& -> void
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

    py::classh<CLS, BaseFixedSizedEdge<D, E, VertexXi>, BaseEdge<D, E>>(
        private_mod_, binary_edge_name.c_str())
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

    pybind11::classh<CLS, BaseFixedSizedEdge<D, E, VertexXi, VertexXj>>(
        private_mod_, binary_edge_name.c_str())
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

    py::classh<CLS, BaseEdge<D, E>>(private_mod_,
                                    base_fixed_size_edge_name.c_str())
        // abstract class type ..."
        // TODO(Rainer): Fix binding of create_vertex
        //  .def("create_vertex", &CLS::createVertex,
        //       "i"_a)  // -> OptimizableGraph::Vertex*
        .def("linearize_oplus",
             static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
        .def("jacobian", &CLS::jacobian, "vertex_index"_a)  // int -> Matrix
        .def("set_jacobian", &CLS::setJacobian, "vertex_index"_a,
             "jacobian"_a)  // int, Matrix ->
        ;
  }

  template <int D, typename E, typename... VertexTypes>
  void registerBaseVariableEdge() {
    this->registerBaseEdge<D, E>();

    using CLS = BaseVariableSizedEdge<D, E>;

    const std::string dim_str = D > 0 ? std::to_string(D) : "Dyn";
    const std::string measurement_str = pybind11::type_id<E>();

    std::stringstream base_variable_edge_name;
    base_variable_edge_name << "BaseVariableEdge_" << dim_str << '_'
                            << measurement_str;
    if (registered_edges_.count(base_variable_edge_name.str()) > 0) {
      return;
    }
    registered_edges_.insert(base_variable_edge_name.str());

    py::classh<CLS, BaseEdge<D, E>>(private_mod_,
                                    base_variable_edge_name.str().c_str())
        .def("linearize_oplus",
             static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
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

    auto base_edge = py::classh<CLS, OptimizableGraph::Edge>(
        private_mod_, base_edge_name.str().c_str());
    base_edge.def("chi2", &CLS::chi2)
        .def_property(
            "error", [](const CLS& cls) { return cls.error(); },
            [](CLS& cls, ErrorVector& v) { cls.error() = v; })
        .def("information", static_cast<InformationType& (CLS::*)()>(
                                &CLS::information))  // -> InformationType
        .def(
            "set_information",
            [](CLS& edge, const MatrixX& info) { edge.setInformation(info); },
            "information"_a)  // InformationType ->
        .def("information_identity", &CLS::informationIdentity)

        .def("dimension_at_compile_time", &CLS::dimensionAtCompileTime)
        .def("measurement", &CLS::measurement)  // -> E
        .def("set_measurement", &CLS::setMeasurement, "m"_a,
             py::keep_alive<1, 2>())  // E ->

        .def("rank", &CLS::rank)  // -> int
        .def("initial_estimate",
             &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                     // OptimizableGraph::Vertex*) ->
        ;

    if constexpr (D == -1) {
      base_edge.def("set_dimension", [](CLS& e, int d) { e.setDimension(d); });
    }
  }

 protected:
  py::module& mod_;
  py::module private_mod_;

  std::unordered_set<std::string> registered_edges_;
  std::unordered_set<std::string> registered_vertices_;
};
}  // namespace g2o::detail
