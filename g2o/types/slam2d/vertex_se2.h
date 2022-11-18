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

#ifndef G2O_VERTEX_SE2_H
#define G2O_VERTEX_SE2_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam2d_api.h"
#include "se2.h"

namespace g2o {

/**
 * \brief 2D pose Vertex, (x,y,theta)
 */
class G2O_TYPES_SLAM2D_API VertexSE2 : public BaseVertex<3, SE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSE2() = default;

  void setToOriginImpl() override { estimate_ = SE2(); }

  void oplusImpl(const VectorX::MapType& update) override {
    Vector2 t = estimate_.translation();
    t += update.head<2>();
    number_t angle = normalize_theta(estimate_.rotation().angle() + update[2]);
    estimate_.setTranslation(t);
    estimate_.setRotation(Rotation2D(angle));
  }

  bool setEstimateDataImpl(const number_t* est) override {
    estimate_ = SE2(est[0], est[1], est[2]);
    return true;
  }

  bool getEstimateData(number_t* est) const override {
    Eigen::Map<Vector3> v(est);
    v = estimate_.toVector();
    return true;
  }

  int estimateDimension() const override { return 3; }

  bool setMinimalEstimateDataImpl(const number_t* est) override {
    return setEstimateData(est);
  }

  bool getMinimalEstimateData(number_t* est) const override {
    return getEstimateData(est);
  }

  int minimalEstimateDimension() const override { return 3; }

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
};

class G2O_TYPES_SLAM2D_API VertexSE2WriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexSE2WriteGnuplotAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      parameters) override;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API VertexSE2DrawAction : public DrawAction {
 public:
  VertexSE2DrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      parameters) override;

 protected:
  HyperGraphElementAction* drawActions_ = nullptr;
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& parameters)
      override;
  std::shared_ptr<FloatProperty> triangleX_, triangleY_;
};
#endif

}  // namespace g2o

#endif
