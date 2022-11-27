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

#include "jacobian_workspace.h"

#include <cmath>

#include "optimizable_graph.h"

namespace g2o {

bool JacobianWorkspace::allocate() {
  // cerr << __PRETTY_FUNCTION__ << " " << PVAR(this) << " " <<
  // PVAR(_maxNumVertices) << " " << PVAR(_maxDimension) << endl;
  if (maxNumVertices_ <= 0 || maxDimension_ <= 0) return false;
  workspace_.resize(maxNumVertices_);
  for (auto& it : workspace_) {
    it.resize(maxDimension_);
    it.setZero();
  }
  return true;
}

void JacobianWorkspace::setZero() {
  for (auto& wp : workspace_) wp.setZero();
}

void JacobianWorkspace::updateSize(const HyperGraph::Edge* e_, bool reset) {
  if (reset) {
    maxNumVertices_ = -1;
    maxDimension_ = -1;
  }

  const auto* e = static_cast<const OptimizableGraph::Edge*>(e_);
  const int errorDimension = e->dimension();
  const int numVertices = e->vertices().size();
  int maxDimensionForEdge = -1;

  for (int i = 0; i < numVertices; ++i) {
    const OptimizableGraph::Vertex* v =
        static_cast<const OptimizableGraph::Vertex*>(e->vertex(i).get());
    assert(v && "Edge has no vertex assigned");
    maxDimensionForEdge =
        std::max(v->dimension() * errorDimension, maxDimensionForEdge);
  }
  maxNumVertices_ = std::max(numVertices, maxNumVertices_);
  maxDimension_ = std::max(maxDimensionForEdge, maxDimension_);
  // cerr << __PRETTY_FUNCTION__ << " " << PVAR(this) << " " <<
  // PVAR(_maxNumVertices) << " " << PVAR(_maxDimension) << endl;
}

void JacobianWorkspace::updateSize(const OptimizableGraph& graph, bool reset) {
  if (reset) {
    maxNumVertices_ = -1;
    maxDimension_ = -1;
  }

  for (const auto& it : graph.edges()) {
    const auto* e = static_cast<const OptimizableGraph::Edge*>(it.get());
    updateSize(e);
  }
}

void JacobianWorkspace::updateSize(int numVertices, int dimension, bool reset) {
  if (reset) {
    maxNumVertices_ = -1;
    maxDimension_ = -1;
  }

  maxNumVertices_ = std::max(numVertices, maxNumVertices_);
  maxDimension_ = std::max(dimension, maxDimension_);
}

}  // namespace g2o
