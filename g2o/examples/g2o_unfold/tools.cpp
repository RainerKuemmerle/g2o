#include "tools.h"

#include <queue>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <limits>
#include <cassert>

#include "g2o/types/slam2d/types_three_dof.h"

namespace g2o{

  using namespace std;

  void findConnectedEdgesWithCostLimit(HyperGraph::EdgeSet& selected,
               HyperGraph::EdgeSet& border,
               HyperGraph::Edge* start,
               HyperDijkstra::CostFunction* cost, 
               double maxEdgeCost, 
               double comparisonConditioner){

    (void) comparisonConditioner; // no warning (unused)
    typedef std::queue<HyperGraph::Edge*> EdgeDeque;
    EdgeDeque frontier;
    frontier.push(start);

    selected.clear();
    border.clear();

    while (! frontier.empty()){
      HyperGraph::Edge* e=frontier.front();
      frontier.pop();

      const VertexSE2* from = dynamic_cast<const VertexSE2*>(e->vertices()[0]);
      const VertexSE2* to   = dynamic_cast<const VertexSE2*>(e->vertices()[1]);

      if (!(from && to))
        continue;

      double edgecost=(*cost)(e, e->vertices()[0], e->vertices()[1]);
      if (edgecost != std::numeric_limits< double >::max()) {

  if (edgecost > maxEdgeCost) {// + comparisonConditioner) {
    border.insert(e);
  }
  else if (edgecost <= maxEdgeCost) {
    selected.insert(e);
    
    for (HyperGraph::EdgeSet::iterator it=e->vertices()[0]->edges().begin(); 
         it!=e->vertices()[0]->edges().end(); ++it) {
      if (selected.find(*it)==selected.end())
        frontier.push(dynamic_cast<HyperGraph::Edge*>(*it));
    }
    for (HyperGraph::EdgeSet::iterator it=e->vertices()[1]->edges().begin(); 
         it!=e->vertices()[1]->edges().end(); ++it) {
      if (selected.find(*it)==selected.end())
        frontier.push(dynamic_cast<HyperGraph::Edge*>(*it));
    }
  }
  else
    cerr << "? nan ?" << endl;
      }
      else
  cerr << "? max ?" << endl;
    }
  }

};
