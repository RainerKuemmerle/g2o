// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "edge_creator.h"

#include "g2o/core/factory.h"

namespace g2o {

bool EdgeCreator::addAssociation(const std::string& vertexTypes,
                                 const std::string& edgeType,
                                 const std::vector<int>& parameterIds) {
  auto it = vertexToEdgeMap_.find(vertexTypes);
  if (it != vertexToEdgeMap_.end())
    it->second = EdgeCreatorEntry(edgeType);
  else
    vertexToEdgeMap_.insert(
        make_pair(vertexTypes, EdgeCreatorEntry(edgeType, parameterIds)));
  return true;
}

bool EdgeCreator::addAssociation(const std::string& vertexTypes,
                                 const std::string& edgeType) {
  return addAssociation(vertexTypes, edgeType, std::vector<int>());
}

bool EdgeCreator::removeAssociation(const std::string& vertexTypes) {
  auto it = vertexToEdgeMap_.find(vertexTypes);
  if (it == vertexToEdgeMap_.end()) return false;
  vertexToEdgeMap_.erase(it);
  return true;
}

std::shared_ptr<OptimizableGraph::Edge> EdgeCreator::createEdge(
    const OptimizableGraph::VertexContainer& vertices) {
  std::stringstream key;
  Factory* factory = Factory::instance();
  for (const auto& vertex : vertices) {
    key << factory->tag(vertex.get()) << ";";
  }
  auto it = vertexToEdgeMap_.find(key.str());
  if (it == vertexToEdgeMap_.end()) {
    std::cerr << "no thing in factory: " << key.str() << std::endl;
    return nullptr;
  }
  std::shared_ptr<HyperGraph::HyperGraphElement> element =
      factory->construct(it->second._edgeTypeName);
  if (!element) {
    std::cerr << "no thing can be created" << std::endl;
    return nullptr;
  }
  std::shared_ptr<OptimizableGraph::Edge> e =
      std::dynamic_pointer_cast<OptimizableGraph::Edge>(element);
  assert(it->second._parameterIds.size() == e->numParameters());
  for (size_t i = 0; i < it->second._parameterIds.size(); i++) {
    if (!e->setParameterId(i, it->second._parameterIds[i])) {
      std::cerr << "no thing in good for setting params" << std::endl;
      return nullptr;
    }
  }
  assert(e);
  for (size_t i = 0; i < vertices.size(); i++) e->vertices()[i] = vertices[i];
  return e;
}
}  // namespace g2o
