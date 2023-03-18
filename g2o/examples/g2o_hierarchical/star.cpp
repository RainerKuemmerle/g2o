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

#include "star.h"

#include "g2o/core/optimization_algorithm_with_hessian.h"

using std::cerr;
using std::endl;

namespace g2o {

Star::Star(int level, SparseOptimizer* optimizer)
    : level_(level), optimizer_(optimizer) {}

bool Star::labelStarEdges(int iterations, EdgeLabeler* labeler) {
  // mark all vertices in the lowLevelEdges as floating
  bool ok = true;
  OptimizableGraph::VertexSet vset;
  for (const auto& _lowLevelEdge : lowLevelEdges_) {
    HyperGraph::Edge* e = _lowLevelEdge.get();
    for (auto& i : e->vertices()) {
      auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(i);
      v->setFixed(false);
      vset.insert(v);
    }
  }
  for (const auto& it : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    v->push();
  }

  // fix all vertices in the gauge
  // cerr << "fixing gauge: ";
  for (const auto& it : gauge_) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    // cerr << v->id() << " ";
    v->setFixed(true);
  }
  // cerr << endl;
  if (iterations > 0) {
    optimizer_->initializeOptimization(lowLevelEdges_);
    optimizer_->computeInitialGuess();
    int result = optimizer_->optimize(iterations);
    if (result < 1) {
      cerr << "Vertices num: " << optimizer_->activeVertices().size()
           << "ids: ";
      for (auto* i : optimizer_->indexMapping()) {
        cerr << i->id() << " ";
      }
      cerr << endl;
      cerr << "!!! optimization failure" << endl;
      cerr << "star size=" << lowLevelEdges_.size() << endl;
      cerr << "gauge: ";
      for (const auto& it : gauge_) {
        auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
        cerr << "[" << v->id() << " " << v->hessianIndex() << "] ";
      }
      cerr << endl;
      ok = false;
    }
  } else {
    optimizer()->initializeOptimization(lowLevelEdges_);
    // cerr << "guess" << endl;
    // optimizer()->computeInitialGuess();
    // cerr << "solver init" << endl;
    optimizer()->solver()->init();
    // cerr << "structure" << endl;
    OptimizationAlgorithmWithHessian* solverWithHessian =
        dynamic_cast<OptimizationAlgorithmWithHessian*>(
            optimizer()->solver().get());
    if (!solverWithHessian->buildLinearStructure())
      cerr << "FATAL: failure while building linear structure" << endl;
    // cerr << "errors" << endl;
    optimizer()->computeActiveErrors();
    // cerr << "system" << endl;
    solverWithHessian->updateLinearSystem();
  }

  std::set<OptimizableGraph::Edge*> star;
  for (const auto& _starEdge : starEdges_) {
    star.insert(static_cast<OptimizableGraph::Edge*>(_starEdge.get()));
  }
  if (ok) {
    int result = labeler->labelEdges(star);
    if (result < 0) ok = false;
  }
  // release all vertices in the gauge
  for (const auto& it : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    v->pop();
  }
  for (const auto& it : gauge_) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    v->setFixed(false);
  }

  return ok;
}

}  // namespace g2o
