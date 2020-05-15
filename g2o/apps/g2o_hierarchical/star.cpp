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

namespace g2o {
  using namespace std;

  Star::Star(int level, SparseOptimizer* optimizer): _level(level), _optimizer(optimizer) {}

  bool Star::labelStarEdges(int iterations, EdgeLabeler* labeler){
    // mark all vertices in the lowLevelEdges as floating
    bool ok=true;
    std::set<OptimizableGraph::Vertex*> vset;
    for (HyperGraph::EdgeSet::iterator it=_lowLevelEdges.begin(); it!=_lowLevelEdges.end(); ++it){
      HyperGraph::Edge* e=*it;
      for (size_t i=0; i<e->vertices().size(); i++){
        OptimizableGraph::Vertex* v=(OptimizableGraph::Vertex*)e->vertices()[i];
        v->setFixed(false);
        vset.insert(v);
      }
    }
    for (std::set<OptimizableGraph::Vertex*>::iterator it=vset.begin(); it!=vset.end(); ++it){
      OptimizableGraph::Vertex* v = *it;
      v->push();
    }

    // fix all vertices in the gauge
    //cerr << "fixing gauge: ";
    for (HyperGraph::VertexSet::iterator it = _gauge.begin(); it!=_gauge.end(); ++it){
      OptimizableGraph::Vertex* v=(OptimizableGraph::Vertex*)*it;
      //cerr << v->id() << " ";
      v->setFixed(true);
    }
    //cerr << endl;
    if (iterations>0){
      _optimizer->initializeOptimization(_lowLevelEdges);
      _optimizer->computeInitialGuess();
      int result=_optimizer->optimize(iterations);
      if (result<1){
        cerr << "Vertices num: " << _optimizer->activeVertices().size() << "ids: ";
        for (size_t i=0; i<_optimizer->indexMapping().size(); i++){
          cerr << _optimizer->indexMapping()[i]->id() << " " ;
        }
        cerr << endl;
        cerr << "!!! optimization failure" << endl;
        cerr << "star size=" << _lowLevelEdges.size() << endl;
        cerr << "gauge: ";
        for (HyperGraph::VertexSet::iterator it=_gauge.begin(); it!=_gauge.end(); ++it){
          OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)*it;
          cerr << "[" << v->id() << " " << v->hessianIndex() << "] ";
        }
        cerr << endl;
        ok=false;
      }
    }  else {
      optimizer()->initializeOptimization(_lowLevelEdges);
      // cerr << "guess" << endl;
      //optimizer()->computeInitialGuess();
      // cerr << "solver init" << endl;
      optimizer()->solver()->init();
      // cerr << "structure" << endl;
      OptimizationAlgorithmWithHessian* solverWithHessian = dynamic_cast<OptimizationAlgorithmWithHessian*> (optimizer()->solver());
      if (!solverWithHessian->buildLinearStructure())
        cerr << "FATAL: failure while building linear structure" << endl;
      // cerr << "errors" << endl;
      optimizer()->computeActiveErrors();
      // cerr << "system" << endl;
      solverWithHessian->updateLinearSystem();
    }

    std::set<OptimizableGraph::Edge*> star;
    for(HyperGraph::EdgeSet::iterator it=_starEdges.begin(); it!=_starEdges.end(); ++it){
      star.insert((OptimizableGraph::Edge*)*it);
    }
    if (ok) {
      int result = labeler->labelEdges(star);
      if (result < 0)
        ok=false;
    }
    // release all vertices in the gauge
    for (std::set<OptimizableGraph::Vertex*>::iterator it=vset.begin(); it!=vset.end(); ++it){
      OptimizableGraph::Vertex* v = *it;
      v->pop();
    }
    for (HyperGraph::VertexSet::iterator it=_gauge.begin(); it!=_gauge.end(); ++it){
      OptimizableGraph::Vertex* v=(OptimizableGraph::Vertex*)*it;
      v->setFixed(false);
    }

    return ok;
  }

}
