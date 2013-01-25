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

#ifndef G2O_EDGE_SE2_PRIOR_XY_H
#define G2O_EDGE_SE2_PRIOR_XY_H

#include "vertex_se2.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  /**
   * \brief Prior for a two D pose with constraints only in xy direction (like gps)
   */
  class G2O_TYPES_SLAM2D_API EdgeSE2XYPrior : public BaseUnaryEdge<2, Eigen::Vector2d, VertexSE2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE2XYPrior();

    virtual bool setMeasurementData(const double* d)
    {
      _measurement[0]=d[0];
      _measurement[1]=d[1];
      return true;
    }

    virtual bool getMeasurementData(double* d) const
    {
      d[0] = _measurement[0];
      d[1] = _measurement[1];
      return true;
    }

    virtual int measurementDimension() const {return 2;}

    virtual void linearizeOplus();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
      
    virtual void computeError()
    {
      const VertexSE2* v = static_cast<const VertexSE2*>(_vertices[0]);
      _error = v->estimate().translation() - _measurement;
    }
  };

} // end namespace

#endif
