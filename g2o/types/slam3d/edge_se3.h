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

#ifndef G2O_EDGE_SE3_H_
#define G2O_EDGE_SE3_H_

#include "g2o/core/base_binary_edge.h"

#include "g2o_types_slam3d_api.h"
#include "vertex_se3.h"

namespace g2o {

  /**
   * \brief Edge between two 3D pose vertices
   *
   * The transformation between the two vertices is given as an Isometry3D.
   * If z denotes the measurement, then the error function is given as follows:
   * z^-1 * (x_i^-1 * x_j)
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3 : public BaseBinaryEdge<6, Isometry3D, VertexSE3, VertexSE3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Isometry3D& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector7d> v(d);
        setMeasurement(internal::fromVectorQT(v));
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector7d> v(d);
        v = internal::toVectorQT(_measurement);
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
      Isometry3D _inverseMeasurement;
  };

  /**
   * \brief Output the pose-pose constraint to Gnuplot data file
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  /**
   * \brief Visualize a 3D pose-pose constraint
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3DrawAction: public DrawAction{
  public:
    EdgeSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace
#endif
