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

#ifndef G2O_EDGE_SE3_LINE_H_
#define G2O_EDGE_SE3_LINE_H_

#include <memory>

#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o_types_slam3d_addons_api.h"
#include "line3d.h"
#include "vertex_line3d.h"

namespace g2o {

class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Line3D
    : public BaseBinaryEdge<4, Line3D, VertexSE3, VertexLine3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeSE3Line3D();

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void computeError() override;

  virtual void setMeasurement(const Vector6& m) { measurement_ = Line3D(m); }

  void setMeasurement(const Line3D& m) override { measurement_ = Line3D(m); }

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector6> v(d);
    measurement_ = Line3D(v);
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector6> v(d);
    v = measurement_;
    return true;
  }

  int measurementDimension() const override { return 6; }

  Vector3 color;

 private:
  std::shared_ptr<CacheSE3Offset> cache_;
  bool resolveCaches() override;
};

#ifdef G2O_HAVE_OPENGL
class EdgeSE3Line3DDrawAction : public DrawAction {
 public:
  G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Line3DDrawAction();
  G2O_TYPES_SLAM3D_ADDONS_API bool operator()(
      HyperGraph::HyperGraphElement& element,
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<FloatProperty> lineLength_, lineWidth_;
};
#endif

}  // namespace g2o
#endif
