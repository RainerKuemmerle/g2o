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

#ifndef G2O_STAR_
#define G2O_STAR_

#include <Eigen/Core>
#include <map>
#include <set>
#include <vector>

#include "edge_labeler.h"
#include "g2o/core/sparse_optimizer.h"

namespace g2o {
/**
 * Class that represents a subgraph in the hierarchical optimization.
 * The subgraph is consisting of
 * <ul>
 * <li> a set of "central" nodes forming the gauge
 * <li> a set of edges and vertices in the lower (denser) level
 * <li> a set of edges in the higher level, each one going from the gauge to one
 * of the vertices in the lower level These edges form the "star" view at the
 * higher level
 * </ul>
 * Additionally, a star provides a function to compute the parameters for
 * each of the edges in the higher level, based on the actual configuration
 * of the state variables.  It does so by using an EdgeLabeler class.
 */

struct Star {
  //! constructs a star at level l in the graph of the sparse optimizer passed
  //! as argument
  //! @param level: the (higher) level of the star
  //! @param optimizer: the optimizer
  Star(int level, SparseOptimizer* optimizer);

  //! labels the edges in the star by first optimizing the low level edges, then
  //! by calling the labelEdge of the labeler.
  //! @param iterations: the number of iterations of the optimizer
  //! @param labeler: the labeler
  bool labelStarEdges(int iterations, EdgeLabeler* labeler);

  //! returns the level of the lower edges in the star
  inline int level() const { return _level; };
  //! returns the optimizer
  inline SparseOptimizer* optimizer() { return _optimizer; }
  //! low level edge set
  inline HyperGraph::EdgeSet& lowLevelEdges() { return _lowLevelEdges; }
  //! high level edge set
  inline HyperGraph::EdgeSet& starEdges() { return _starEdges; }
  //! edges in the high level that lead to some node owned by a different star
  inline HyperGraph::EdgeSet& starFrontierEdges() { return _starFrontierEdges; }
  //! set of nodes to keep fixed in the optimization
  inline HyperGraph::VertexSet& gauge() { return _gauge; }
  //! set of all vertices in the low level
  inline HyperGraph::VertexSet& lowLevelVertices() { return _lowLevelVertices; }

  //! level of the star
  int _level;
  //! optimizer
  SparseOptimizer* _optimizer;
  //! edges in the lower level
  HyperGraph::EdgeSet _lowLevelEdges;
  //! edges in the star
  HyperGraph::EdgeSet _starEdges;
  //! edges in the star that lead to some other star
  HyperGraph::EdgeSet _starFrontierEdges;
  //! vertices that are fixed (center of the star)
  HyperGraph::VertexSet _gauge;
  //! vertices that are fixed (center of the star)
  HyperGraph::VertexSet _lowLevelVertices;
};

typedef std::multimap<OptimizableGraph::Vertex*, Star*> VertexStarMultimap;
typedef std::map<OptimizableGraph::Vertex*, Star*> VertexStarMap;
typedef std::set<Star*> StarSet;
typedef std::map<HyperGraph::Edge*, Star*> EdgeStarMap;

}  // namespace g2o
#endif
