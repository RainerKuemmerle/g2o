#pragma once

#include <pybind11/detail/typeid.h>

#include <sstream>
#include <string>

#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"
#include "python/trampoline/py_edge_trampoline.h"

namespace g2o {

template <typename EdgeType>
void registerEdgeBinary(py::module& m, const char* name) {
  const std::string dim_str =
      EdgeType::kDimension > 0 ? std::to_string(EdgeType::kDimension) : "Dyn";
  const std::string measurement_str =
      pybind11::type_id<typename EdgeType::Measurement>();
  const std::string vi_str =
      pybind11::type_id<typename EdgeType::VertexXiType>();
  const std::string vj_str =
      pybind11::type_id<typename EdgeType::VertexXjType>();

  std::stringstream base_binary_name;
  base_binary_name << '_' << dim_str << '_' << measurement_str << '_' << vi_str
                   << '_' << vj_str;

  templatedBaseBinaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                          typename EdgeType::VertexXiType,
                          typename EdgeType::VertexXjType>(
      m, base_binary_name.str());

  py::class_<
      EdgeType,
      BaseBinaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                     typename EdgeType::VertexXiType,
                     typename EdgeType::VertexXjType>,
      PyEdgeTrampoline<EdgeType>, std::shared_ptr<EdgeType>>(m, name)
      .def(py::init<>());
}

}  // namespace g2o
