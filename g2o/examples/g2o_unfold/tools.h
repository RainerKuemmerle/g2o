#ifndef G2O_AIS_GRAPH_TOOLS_HH
#define G2O_AIS_GRAPH_TOOLS_HH

#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_dijkstra.h"

namespace g2o{

 void findConnectedEdgesWithCostLimit(HyperGraph::EdgeSet& selected,
              HyperGraph::EdgeSet& border,
              HyperGraph::Edge* start,
              HyperDijkstra::CostFunction* cost, 
              double maxEdgeCost, 
              double comparisonConditioner=1e-6);
 };

#endif


