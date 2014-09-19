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

#ifndef G2O_EDGE_SE2_XY_CALIB_H
#define G2O_EDGE_SE2_XY_CALIB_H

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  /**
   * \brief Landmark measurement that also calibrates an offset for the landmark measurement
   */
  class G2O_TYPES_SLAM2D_API EdgeSE2PointXYCalib : public BaseMultiEdge<2, Vector2D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2PointXYCalib();

      void computeError()
      {
        const VertexSE2* v1     = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
        const VertexSE2* calib  = static_cast<const VertexSE2*>(_vertices[2]);
        _error = ((v1->estimate() * calib->estimate()).inverse() * l2->estimate()) - _measurement;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
  };

}

#endif
