// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "edge_se2_pointxy_calib.h"

namespace g2o {

  EdgeSE2PointXYCalib::EdgeSE2PointXYCalib() :
    BaseMultiEdge<2, Vector2d>()
  {
    resize(3);
  }

  void EdgeSE2PointXYCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexPointXY");

    if (from.count(_vertices[0]) != 1)
      return;
    VertexSE2* vi     = static_cast<VertexSE2*>(_vertices[0]);
    VertexPointXY* vj = static_cast<VertexPointXY*>(_vertices[1]);
    vj->setEstimate(vi->estimate() * _measurement);
  }

  bool EdgeSE2PointXYCalib::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0,0) >> information()(0,1) >> information()(1,1);
    information()(1,0) = information()(0,1);
    return true;
  }

  bool EdgeSE2PointXYCalib::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
    return os.good();
  }

} // end namespace
