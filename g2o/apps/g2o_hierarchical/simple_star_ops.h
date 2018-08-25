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

#ifndef G2O_SIMPLE_STAR_OPS_
#define G2O_SIMPLE_STAR_OPS_

#include <string>
#include <map>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"
#include "star.h"
#include "edge_creator.h"
#include "edge_labeler.h"

#include "g2o_hierarchical_api.h"

namespace g2o {


  G2O_HIERARCHICAL_API void constructEdgeStarMap(EdgeStarMap& esmap, StarSet& stars, bool low=true);

  size_t vertexEdgesInStar(HyperGraph::EdgeSet& eset, HyperGraph::Vertex* v, Star* s, EdgeStarMap& esmap);

  void starsInVertex(StarSet& stars, HyperGraph::Vertex* v, EdgeStarMap& esmap);

  void assignHierarchicalEdges(StarSet& stars, EdgeStarMap& esmap, EdgeLabeler* labeler, EdgeCreator* creator, SparseOptimizer* optimizer, int minNumEdges, int maxIterations);

  G2O_HIERARCHICAL_API void computeBorder(StarSet& stars, EdgeStarMap& hesmap);

  G2O_HIERARCHICAL_API void computeSimpleStars(StarSet& stars,
                          SparseOptimizer* optimizer,
                          EdgeLabeler* labeler,
                          EdgeCreator* creator,
                          OptimizableGraph::Vertex* gauge_,
                          std::string edgeTag, std::string vertexTag,
                          int level,
                          int step,
                          int backboneIterations=1,
                          int starIterations=30,
                          double rejectionThreshold=1e-5, bool debug=false);

}
#endif
