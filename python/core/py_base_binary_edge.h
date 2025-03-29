#pragma once
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2opy.h"
#include "python/core/py_base_fixed_sized_edge.h"

namespace g2o {

template <int D, typename E, typename VertexXi, typename VertexXj>
void templatedBaseBinaryEdge(pybind11::module& m, const std::string& suffix) {
  using CLS = BaseBinaryEdge<D, E, VertexXi, VertexXj>;

  templatedBaseFixedSizedEdge<D, E, VertexXi, VertexXj>(m, suffix);

  pybind11::class_<CLS, BaseFixedSizedEdge<D, E, VertexXi, VertexXj>,
                   BaseEdge<D, E>, std::shared_ptr<CLS>>(
      m, ("BaseBinaryEdge" + suffix).c_str())
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

}  // end namespace g2o
