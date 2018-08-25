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
