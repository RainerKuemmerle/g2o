// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_CORE_IO_WRAPPER_JSON_H
#define G2O_CORE_IO_WRAPPER_JSON_H

#include "g2o/config.h"

#ifdef G2O_HAVE_JSON
#include <nlohmann/json.hpp>

#include "g2o/core/abstract_graph.h"

namespace g2o {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AbstractGraph::AbstractParameter, tag, id,
                                   value);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AbstractGraph::AbstractData, tag, data);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AbstractGraph::AbstractVertex, tag, id,
                                   estimate, data);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AbstractGraph::AbstractEdge, tag, ids,
                                   param_ids, measurement, information, data);

namespace json {

/**
 * @brief Versions of the IO API for the graph using cereal.
 */
enum class IoVersions {
  kGraph = 0,
  kData = 0,
  kParameter = 0,
  kGraphElement = 0,
  kVertex = 0,
  kEdge = 0,
};

inline AbstractGraph fromJson(const nlohmann::json& json) {
  const nlohmann::json& json_graph = json["graph"];
  AbstractGraph graph;
  graph.fixed() = json_graph["fixed"].get<std::vector<int>>();
  graph.parameters() =
      json_graph["params"].get<std::vector<AbstractGraph::AbstractParameter>>();
  graph.vertices() =
      json_graph["vertices"].get<std::vector<AbstractGraph::AbstractVertex>>();
  graph.edges() =
      json_graph["edges"].get<std::vector<AbstractGraph::AbstractEdge>>();
  return graph;
}

inline nlohmann::json toJson(const AbstractGraph& graph) {
  nlohmann::json json;
  nlohmann::json& json_graph = json["graph"];
  json_graph["fixed"] = graph.fixed();
  json_graph["params"] = graph.parameters();
  json_graph["vertices"] = graph.vertices();
  json_graph["edges"] = graph.edges();
  return json;
}

}  // namespace json

}  // namespace g2o

#endif  // HAVE_JSON

#endif
