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

#ifndef G2O_CORE_IO_WARPPER_CEREAL_H
#define G2O_CORE_IO_WARPPER_CEREAL_H

#include "g2o/config.h"

#ifdef G2O_HAVE_CEREAL

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <exception>
#include <optional>
#include <string_view>

#include "g2o/core/abstract_graph.h"
#include "g2o/stuff/logger.h"

namespace g2o {

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

template <class Archive>
void serialize(Archive& archive, AbstractGraph& graph,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp("fixed", graph.fixed()),
          cereal::make_nvp("params", graph.parameters()),
          cereal::make_nvp("vertices", graph.vertices()),
          cereal::make_nvp("edges", graph.edges()));
}

template <class Archive>
void serialize(Archive& archive, AbstractGraph::AbstractData& data,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp("tag", data.tag),
          cereal::make_nvp("data", data.data));
}

template <class Archive>
void serialize(Archive& archive, AbstractGraph::AbstractParameter& param,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp("tag", param.tag), cereal::make_nvp("id", param.id),
          cereal::make_nvp("value", param.value));
}

template <class Archive>
void serialize(Archive& archive, AbstractGraph::AbstractGraphElement& elem,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp("tag", elem.tag),
          cereal::make_nvp("data", elem.data));
}

template <class Archive>
void serialize(Archive& archive, AbstractGraph::AbstractVertex& vertex,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp(
              "elem",
              cereal::base_class<AbstractGraph::AbstractGraphElement>(&vertex)),
          cereal::make_nvp("id", vertex.id),
          cereal::make_nvp("estimate", vertex.estimate));
}

template <class Archive>
void serialize(Archive& archive, AbstractGraph::AbstractEdge& edge,
               const std::uint32_t version) {
  (void)version;
  archive(cereal::make_nvp(
              "elem",
              cereal::base_class<AbstractGraph::AbstractGraphElement>(&edge)),
          cereal::make_nvp("ids", edge.ids),
          cereal::make_nvp("param_ids", edge.param_ids),
          cereal::make_nvp("measurement", edge.measurement),
          cereal::make_nvp("information", edge.information));
}

namespace io {
template <typename ArchiveType>
std::optional<AbstractGraph> load(std::istream& input, std::string_view name) {
  try {
    ArchiveType archive(input);
    AbstractGraph result;
    archive(cereal::make_nvp("graph", result));
    return result;
  } catch (const std::exception& e) {
    G2O_ERROR("Exception while loading {}: {}", name, e.what());
  }
  return std::nullopt;
}

template <typename ArchiveType>
bool save(std::ostream& output, const AbstractGraph& graph,
          std::string_view name) {
  try {
    ArchiveType archive(output);
    archive(cereal::make_nvp("graph", graph));
    return true;
  } catch (const std::exception& e) {
    G2O_ERROR("Exception while saving {}: {}", name, e.what());
  }
  return false;
}

}  // namespace io

}  // namespace g2o

// Register Version numbers
CEREAL_CLASS_VERSION(g2o::AbstractGraph,
                     static_cast<std::uint32_t>(g2o::IoVersions::kGraph));
CEREAL_CLASS_VERSION(g2o::AbstractGraph::AbstractData,
                     static_cast<std::uint32_t>(g2o::IoVersions::kData));
CEREAL_CLASS_VERSION(g2o::AbstractGraph::AbstractParameter,
                     static_cast<std::uint32_t>(g2o::IoVersions::kParameter));
CEREAL_CLASS_VERSION(
    g2o::AbstractGraph::AbstractGraphElement,
    static_cast<std::uint32_t>(g2o::IoVersions::kGraphElement));
CEREAL_CLASS_VERSION(g2o::AbstractGraph::AbstractVertex,
                     static_cast<std::uint32_t>(g2o::IoVersions::kVertex));
CEREAL_CLASS_VERSION(g2o::AbstractGraph::AbstractEdge,
                     static_cast<std::uint32_t>(g2o::IoVersions::kEdge));

#endif  // HAVE_CEREAL

#endif
