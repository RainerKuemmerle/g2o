#ifndef G2O_SIMPLE_STAR_OPS_
#define G2O_SIMPLE_STAR_OPS_

#include <string>
#include <map>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"
#include "star.h"
#include "edge_creator.h"
#include "edge_labeler.h"


namespace g2o {


  void constructEdgeStarMap(EdgeStarMap& esmap, StarSet& stars, bool low=true);

  size_t vertexEdgesInStar(HyperGraph::EdgeSet& eset, HyperGraph::Vertex* v, Star* s, EdgeStarMap& esmap);

  void starsInVertex(StarSet& stars, HyperGraph::Vertex* v, EdgeStarMap& esmap);

  void assignHierarchicalEdges(StarSet& stars, EdgeStarMap& esmap, EdgeLabeler* labeler, EdgeCreator* creator, SparseOptimizer* optimizer, int minNumEdges, int maxIterations);

  void computeBorder(StarSet& stars, EdgeStarMap& hesmap);

  void computeSimpleStars(StarSet& stars,
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
