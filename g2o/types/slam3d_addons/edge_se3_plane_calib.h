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

#ifndef G2O_EDGE_SE3_PLANE_CALIB_H
#define G2O_EDGE_SE3_PLANE_CALIB_H

#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "vertex_plane.h"
#include "g2o/config.h"

namespace g2o {
  /**
   * \brief plane measurement that also calibrates an offset for the sensor
   */
  class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalib : public BaseMultiEdge<3, Plane3D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3PlaneSensorCalib();
      Vector3 color;

      void computeError()
      {
        const VertexSE3* v1            = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[1]);
        const VertexSE3* offset        = static_cast<const VertexSE3*>(_vertices[2]);
        const Plane3D& plane           = planeVertex->estimate();
	// measurement function: remap the plane in global coordinates
        Isometry3 w2n=(v1->estimate()*offset->estimate()).inverse();
	Plane3D localPlane=w2n*plane;
	_error = localPlane.ominus(_measurement);
      }

      void setMeasurement(const Plane3D& m){
	_measurement = m;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };

#ifdef G2O_HAVE_OPENGL
  class EdgeSE3PlaneSensorCalibDrawAction: public DrawAction{
  public:
    G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalibDrawAction();
    G2O_TYPES_SLAM3D_ADDONS_API virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _planeWidth, *_planeHeight;
  };
#endif

} // end namespace

#endif
