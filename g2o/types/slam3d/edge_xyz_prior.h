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

#ifndef G2O_EDGE_XYZ_PRIOR_H_
#define G2O_EDGE_XYZ_PRIOR_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam3d_api.h"
#include "vertex_pointxyz.h"

namespace g2o {
  /**
   * \brief prior for an XYZ vertex (VertexPointXYZ)
   *
   * Provides a prior for a 3d point vertex. The measurement is represented by a
   * Vector3 with a corresponding 3x3 upper triangle covariance matrix (upper triangle only).
   */
  class G2O_TYPES_SLAM3D_API EdgeXYZPrior : public BaseUnaryEdge<3, Vector3, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeXYZPrior();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError();

    // jacobian
    virtual void linearizeOplus();

    virtual bool setMeasurementData(const number_t* d){
        Eigen::Map<const Vector3> v(d);
        _measurement = v;
        return true;
    }

    virtual bool getMeasurementData(number_t* d) const{
        Eigen::Map<Vector3> v(d);
        v = _measurement;
        return true;
    }

    virtual int measurementDimension() const { return 3; }

    virtual bool setMeasurementFromState() ;

    virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
             OptimizableGraph::Vertex* /*to*/) {
      return 0;
    }
  };

}
#endif
