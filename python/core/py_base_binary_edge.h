#pragma once
#include "g2o/core/base_binary_edge.h"
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
      //.def(pybind11::init<>())    // lead to "error:
      /*
      .def_readwrite("jacobian_oplus_xi",
                     &CLS::template jacobianOplusXn<0>())  // ->
      JacobianXiOplusType& .def_readwrite("jacobian_oplus_xj", &CLS::template
      jacobianOplusXn<1>())  //-> JacobianXjOplusType&
      */
      ;
}

}  // end namespace g2o
