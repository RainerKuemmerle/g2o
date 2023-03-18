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

#ifndef G2O_BACKBONE_TREE_ACTION_
#define G2O_BACKBONE_TREE_ACTION_

#include <map>
#include <string>

#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/sparse_optimizer.h"
#include "star.h"

namespace g2o {

/**
 * Dijkstra traversal action that constructs a backbone skeleton on the
 * graph.  It assumes that the traversal contains only edges and vertices
 * belonging to classes that can be used to construct a backbone.  After a
 * visit is invoked, it returns vector of stars to which the backbone nodes
 * have been assigned
 */
struct BackBoneTreeAction : public HyperDijkstra::TreeAction {
  /**
   * creates a tree action for constructing the backbone
   * @param optimizer: the optimizer on which the stars are constructed
   * @param vertexTag: the tag of the vertices to use as backbone nodes
   * @param level: the level of the lowLevelEdges of the stars in the backbone
   * @param step:  depth in tree after which a new star is initialized
   */
  BackBoneTreeAction(SparseOptimizer* optimizer, std::string vertexTag,
                     int level, int step);

  //! initializes the visit and clears the internal structures
  void init();

  //! map vertex->star. Contains the most recent vertex assignment
  inline VertexStarMap& vertexStarMap() { return vsMap_; }
  //! multimap vertex->star. Contains all the vertex assignments to all stars
  inline VertexStarMultimap& vertexStarMultiMap() { return vsMmap_; }
  //! edges that are not yet assigned to any star
  inline HyperGraph::EdgeSet& freeEdges() { return freeEdges_; }

  /**
   * action to be performed during the descent of the dijkstra tree.  it
   * constructs stars according to the dijkstra tree A new star is created
   * every time the depth increases of _step.
   * @param v: the vertex
   * @param vParent: the parent vertex
   * @param e: the edge between v and its parent
   * @param distance: the depth in the tree
   */
  double perform(const std::shared_ptr<HyperGraph::Vertex>& v,
                 const std::shared_ptr<HyperGraph::Vertex>& vParent,
                 const std::shared_ptr<HyperGraph::Edge>& e,
                 double distance) override;

 protected:
  /**
   * helper function for adding a vertex to a star. If the vertex is already
   * in _vertexToEdgeMap it replaces the associated star. _vertexStarMultimap
   * is only augmented, thus it contains all associations
   * @param s: the star
   * @param v: the vertex
   */
  void addToMap(const std::shared_ptr<Star>& s,
                const std::shared_ptr<OptimizableGraph::Vertex>& v);
  //! helper function to retrieve the most recent star of a vertex.
  //! @param v: the vertex
  std::shared_ptr<Star> getStar(
      const std::shared_ptr<OptimizableGraph::Vertex>& v);

  //! helper function that adds to a star an edge and all its vertices
  bool fillStar(const std::shared_ptr<Star>& s,
                const std::shared_ptr<OptimizableGraph::Edge>& e_);

  SparseOptimizer* optimizer_;
  std::string vertexTag_;
  int level_;
  int step_;

  VertexStarMap vsMap_;
  VertexStarMultimap vsMmap_;
  HyperGraph::EdgeSet freeEdges_;
  Factory* factory_;
};

}  // namespace g2o
#endif
