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
namespace internal {
/**
 * @brief Get the object from the JSON if key exists
 *
 * @tparam T type of the target value
 * @param j a JSON
 * @param key key for retrieving the value
 * @param target Where to store the value
 */
template <typename T>
void get_to_if_exists(const nlohmann::json& j, const char* key, T& target) {
  auto it = j.find(key);
  if (it == j.end()) return;
  it->get_to(target);
}

/**
 * @brief Store a container into the JSON if not empty
 *
 * @tparam T type of the container
 * @param j a JSON
 * @param key key for storing the container
 * @param value the container to store
 */
template <typename T>
void store_if_not_empty(nlohmann::json& j, const char* key, const T& value) {
  if (value.empty()) return;
  j[key] = value;
}
}  // namespace internal

// PARAMETER
inline void to_json(nlohmann::json& j,
                    const AbstractGraph::AbstractParameter& param) {
  j = nlohmann::json{
      {"tag", param.tag}, {"id", param.id}, {"value", param.value}};
}

inline void from_json(const nlohmann::json& j,
                      AbstractGraph::AbstractParameter& param) {
  j.at("tag").get_to(param.tag);
  j.at("id").get_to(param.id);
  j.at("value").get_to(param.value);
}

// DATA
inline void to_json(nlohmann::json& j,
                    const AbstractGraph::AbstractData& param) {
  j = nlohmann::json{{"tag", param.tag}, {"data", param.data}};
}

inline void from_json(const nlohmann::json& j,
                      AbstractGraph::AbstractData& param) {
  j.at("tag").get_to(param.tag);
  j.at("data").get_to(param.data);
}

// VERTEX
inline void to_json(nlohmann::json& j,
                    const AbstractGraph::AbstractVertex& vertex) {
  j = nlohmann::json{
      {"tag", vertex.tag}, {"id", vertex.id}, {"estimate", vertex.estimate}};
  internal::store_if_not_empty(j, "data", vertex.data);
}

inline void from_json(const nlohmann::json& j,
                      AbstractGraph::AbstractVertex& vertex) {
  j.at("tag").get_to(vertex.tag);
  j.at("id").get_to(vertex.id);
  j.at("estimate").get_to(vertex.estimate);
  internal::get_to_if_exists(j, "data", vertex.data);
}

// EDGE
inline void to_json(nlohmann::json& j,
                    const AbstractGraph::AbstractEdge& edge) {
  j = nlohmann::json{{"tag", edge.tag},
                     {"ids", edge.ids},
                     {"measurement", edge.measurement},
                     {"information", edge.information}};
  internal::store_if_not_empty(j, "data", edge.data);
  internal::store_if_not_empty(j, "param_ids", edge.param_ids);
}

inline void from_json(const nlohmann::json& j,
                      AbstractGraph::AbstractEdge& edge) {
  j.at("tag").get_to(edge.tag);
  j.at("ids").get_to(edge.ids);
  j.at("measurement").get_to(edge.measurement);
  j.at("information").get_to(edge.information);
  internal::get_to_if_exists(j, "data", edge.data);
  internal::get_to_if_exists(j, "param_ids", edge.param_ids);
}

namespace json {

/**
 * @brief Versions of the IO API for the graph. Currently not stored.
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
  json_graph["vertices"].get_to(graph.vertices());
  json_graph["edges"].get_to(graph.edges());
  internal::get_to_if_exists(json_graph, "fixed", graph.fixed());
  internal::get_to_if_exists(json_graph, "params", graph.parameters());
  return graph;
}

inline nlohmann::json toJson(const AbstractGraph& graph) {
  nlohmann::json json;
  nlohmann::json& json_graph = json["graph"];
  json_graph["vertices"] = graph.vertices();
  json_graph["edges"] = graph.edges();
  g2o::internal::store_if_not_empty(json_graph, "fixed", graph.fixed());
  g2o::internal::store_if_not_empty(json_graph, "params", graph.parameters());
  return json;
}

}  // namespace json

}  // namespace g2o

#endif  // HAVE_JSON

#endif
