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

#ifndef G2O_DEPRECATED_EDGE_SE3_OFFSET_H_
#define G2O_DEPRECATED_EDGE_SE3_OFFSET_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se3_quat.h"

namespace g2o {
namespace deprecated {

  class ParameterSE3Offset;
  class CacheSE3Offset;

  /**
   * \brief Offset edge
   */
  // first two args are the measurement type, second two the connection classes
  class G2O_DEPRECATED_TYPES_SLAM3D_API EdgeSE3Offset : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Offset();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      // jacobian
      //virtual void linearizeOplus();

      virtual void setMeasurement(const SE3Quat& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector7d> v(d);
        _measurement.fromVector(v);
        _inverseMeasurement = _measurement.inverse();
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector7d> v(d);
        v = _measurement.toVector();
        return true;
      }

      void linearizeOplus();

      virtual int measurementDimension() const {return 7;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      SE3Quat _inverseMeasurement;
      virtual bool resolveCaches();
      ParameterSE3Offset *_offsetFrom, *_offsetTo;
      CacheSE3Offset  *_cacheFrom, *_cacheTo;
  };

} // end namespace
} // end namespace
#endif
