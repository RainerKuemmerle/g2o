#pragma once

#include "g2o/core/base_unary_edge.h"
#include "g2opy.h"
#include "py_base_fixed_sized_edge.h"

namespace g2o {

template <int D, typename E, typename VertexXiType>
void templatedBaseUnaryEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseUnaryEdge<D, E, VertexXiType>;

  templatedBaseFixedSizedEdge<D, E, VertexXiType>(m, suffix);

  py::class_<CLS, BaseFixedSizedEdge<D, E, VertexXiType>, BaseEdge<D, E>,
             std::shared_ptr<CLS>>(m, ("BaseUnaryEdge" + suffix).c_str())
      /*
      .def_readwrite("jacobian_oplus_xi",
                     &CLS::template jacobianOplusXn<0>())  // ->
      JacobianXiOplusType&
      */
      ;
}

}  // end namespace g2o
