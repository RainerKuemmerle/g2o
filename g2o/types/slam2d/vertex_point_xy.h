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

#ifndef G2O_VERTEX_POINT_XY_H
#define G2O_VERTEX_POINT_XY_H

#include <Eigen/Core>

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API VertexPointXY : public BaseVertex<2, Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexPointXY();

  void setToOriginImpl() override { estimate_.setZero(); }

  bool setEstimateDataImpl(const number_t* est) override {
    estimate_[0] = est[0];
    estimate_[1] = est[1];
    return true;
  }

  bool getEstimateData(number_t* est) const override {
    est[0] = estimate_[0];
    est[1] = estimate_[1];
    return true;
  }

  int estimateDimension() const override { return 2; }

  bool setMinimalEstimateDataImpl(const number_t* est) override {
    return setEstimateData(est);
  }

  bool getMinimalEstimateData(number_t* est) const override {
    return getEstimateData(est);
  }

  int minimalEstimateDimension() const override { return 2; }

  void oplusImpl(const VectorX::MapType& update) override {
    estimate_ += update.head<2>();
  }

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
};

class G2O_TYPES_SLAM2D_API VertexPointXYWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexPointXYWriteGnuplotAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params) override;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API VertexPointXYDrawAction : public DrawAction {
 public:
  VertexPointXYDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params) override;

 protected:
  std::shared_ptr<FloatProperty> pointSize_;
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params)
      override;
};
#endif

}  // namespace g2o

#endif
