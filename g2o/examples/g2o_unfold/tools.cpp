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
