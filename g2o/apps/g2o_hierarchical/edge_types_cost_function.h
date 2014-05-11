#ifndef G2O_EDGE_TYPES_COST_FUNCTION_
#define G2O_EDGE_TYPES_COST_FUNCTION_

#include <string>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"

namespace g2o {

  /**
   * Cost function for Hyper-Dijkstra that returns 1 when for edges that belong
   * to a given type and maxdouble otherwise.  It can be used to construct a
   * backbone of a hierarchical graph by running Dijkstra.
   */
struct EdgeTypesCostFunction: public HyperDijkstra::CostFunction {
  //! creates a cost function that matches edges at a given level, whose tag is the one given and that are leaving/leading to vertices
  //! of a selected type.
  //! @param edgeTag: the tag of the edge type to consider
  //! @param vertexTag: the tag of the vertex to  consider
  //! @param level: the level of the edge
  EdgeTypesCostFunction(std::string edgeTag, std::string vertexTag, int level);

  //!cost operator
  virtual double operator() (HyperGraph::Edge* e_, HyperGraph::Vertex* from, HyperGraph::Vertex* to);

  std::string _edgeTag;
  std::string _vertexTag;
  Factory* _factory;
  int _level;
};

}
#endif
