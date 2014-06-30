#ifndef G2O_STAR_
#define G2O_STAR_

#include "g2o/core/sparse_optimizer.h"
#include "edge_labeler.h"
#include <Eigen/Core>
#include <vector>
#include <set>
#include <map>

namespace g2o {
  /**
   * Class that represents a subgraph in the hierarchical optimization.
   * The subgraph is consisting of
   * <ul>
   * <li> a set of "central" nodes forming the gauge
   * <li> a set of edges and vertices in the lower (denser) level
   * <li> a set of edges in the higher level, each one going from the gauge to one of the vertices in the lower level
   * These edges form the "star" view at the higher level
   * </ul>
   * Additionally, a star provides a function to compute the parameters for
   * each of the edges in the higher level, based on the actual configuration
   * of the state variables.  It does so by using an EdgeLabeler class.
   */

struct Star{
  //! constructs a star at level l in the graph of the sparse optimizer passed as argument
  //! @param level: the (higher) level of the star
  //! @param optimizer: the optimizer
  Star(int level, SparseOptimizer* optimizer);

  //! labels the edges in the star by first optimizing the low level edges, then by
  //! calling the labelEdge of the labeler.
  //! @param iterations: the number of iterations of the optimizer
  //! @param labeler: the labeler
  bool labelStarEdges(int iterations, EdgeLabeler* labeler);

  //! returns the level of the lower edges in the star
  inline int level() const { return _level; };
  //! returns the optimizer
  inline SparseOptimizer* optimizer() { return _optimizer;}
  //! low level edge set
  inline HyperGraph::EdgeSet& lowLevelEdges() {return _lowLevelEdges;}
  //! high level edge set
  inline HyperGraph::EdgeSet& starEdges() {return _starEdges;}
  //! edges in the high level that lead to some node owned by a different star
  inline HyperGraph::EdgeSet& starFrontierEdges() {return _starFrontierEdges;}
  //! set of nodes to keep fixed in the optimization
  inline HyperGraph::VertexSet& gauge() {return _gauge;}
  //! set of all vertices in the low level
  inline HyperGraph::VertexSet& lowLevelVertices() {return _lowLevelVertices;}

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


} // end namespace
#endif
