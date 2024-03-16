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

#include <memory>

#include "g2o/config.h"
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/stuff/property.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h"
#include "g2o/types/slam3d_addons/plane3d.h"
#include "vertex_plane.h"

namespace g2o {
/**
 * \brief plane measurement that also calibrates an offset for the sensor
 */
class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalib
    : public BaseVariableSizedEdge<3, Plane3D> {
 public:
  EdgeSE3PlaneSensorCalib();
  Vector3 color;

  void computeError() override {
    const auto* v1 = static_cast<const VertexSE3*>(vertexRaw(0));
    const auto* planeVertex = static_cast<const VertexPlane*>(vertexRaw(1));
    const auto* offset = static_cast<const VertexSE3*>(vertexRaw(2));
    const Plane3D& plane = planeVertex->estimate();
    // measurement function: remap the plane in global coordinates
    Isometry3 w2n = (v1->estimate() * offset->estimate()).inverse();
    Plane3D localPlane = w2n * plane;
    error_ = localPlane.ominus(measurement_);
  }

  void setMeasurement(const Plane3D& m) override { measurement_ = m; }
};

#ifdef G2O_HAVE_OPENGL
class EdgeSE3PlaneSensorCalibDrawAction : public DrawAction {
 public:
  G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalibDrawAction();
  G2O_TYPES_SLAM3D_ADDONS_API bool operator()(
      HyperGraph::HyperGraphElement& element,
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<FloatProperty> planeWidth_, planeHeight_;
};
#endif

}  // namespace g2o

#endif
