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

#ifndef G2O_VERTEX_LINE3D_H_
#define G2O_VERTEX_LINE3D_H_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam3d_addons_api.h"
#include "line3d.h"

namespace g2o {

class G2O_TYPES_SLAM3D_ADDONS_API VertexLine3D : public BaseVertex<4, Line3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexLine3D();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setToOriginImpl() override { estimate_ = Line3D(); }

  void oplusImpl(const VectorX::MapType& update) override {
    estimate_.oplus(update);
  }

  bool setEstimateDataImpl(const number_t* est) override {
    Eigen::Map<const Vector6> _est(est);
    estimate_ = Line3D(_est);
    return true;
  }

  bool getEstimateData(number_t* est) const override {
    Eigen::Map<Vector6> _est(est);
    _est = estimate_;
    return true;
  }

  int estimateDimension() const override { return 6; }

  Vector3 color;
};

#ifdef G2O_HAVE_OPENGL
class VertexLine3DDrawAction : public DrawAction {
 public:
  VertexLine3DDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<FloatProperty> lineLength_, lineWidth_;
};
#endif

}  // namespace g2o
#endif
