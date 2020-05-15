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

#include <limits>
#include "g2o/core/factory.h"
#include "edge_types_cost_function.h"

namespace g2o {

  EdgeTypesCostFunction::EdgeTypesCostFunction(const std::string& edgeTag, const std::string& vertexTag, int level):
    _edgeTag(edgeTag),
    _vertexTag(vertexTag),
    _factory(Factory::instance()),
    _level(level)
  {}

  double EdgeTypesCostFunction::operator() (HyperGraph::Edge* e_, HyperGraph::Vertex* from, HyperGraph::Vertex* to){
    OptimizableGraph::Edge*e =(OptimizableGraph::Edge*)(e_);
    if (e->level()==_level && _factory->tag(e)==_edgeTag && _factory->tag(from)==_vertexTag && _factory->tag(to)==_vertexTag) {
      return 1.;
    }
    return std::numeric_limits<double>::max();
  }

}
