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

#ifndef G2O_EDGE_PLANE_NORMALIZE_H_
#define G2O_EDGE_PLANE_NORMALIZE_H_

#include "vertex_plane_nm.h"
#include "g2o/core/base_unary_edge.h"
namespace Slam3dAddons {
  using namespace g2o;
  /**
   * \brief prior for an SE3 element
   *
   * Provides a prior for a 3d pose vertex. Again the measurement is represented by an
   * Isometry3d matrix.
   */
  class EdgePlaneNormalize : public BaseUnaryEdge<1, double, VertexPlaneNM> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePlaneNormalize();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError() {
      const VertexPlaneNM* vPlane = dynamic_cast<const VertexPlaneNM*>(_vertices[0]);
      _error[0] = vPlane->estimate().coeffs().head<3>().squaredNorm() - _measurement;
    }
    
    // // jacobian
    // virtual void linearizeOplus();

    virtual void setMeasurement(const double& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const double* d){
      _measurement = *d;
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      *d = _measurement;
      return true;
    }
    
    virtual int measurementDimension() const {return 1;}

    //virtual bool setMeasurementFromState() ;

  protected:
  };

}
#endif
