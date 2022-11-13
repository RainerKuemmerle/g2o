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

#ifndef G2O_VERTEX_TRACKXYZ_H_
#define G2O_VERTEX_TRACKXYZ_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam3d_api.h"

namespace g2o {
/**
 * \brief Vertex for a tracked point in space
 */
class G2O_TYPES_SLAM3D_API VertexPointXYZ : public BaseVertex<3, Vector3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexPointXYZ() = default;
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setToOriginImpl() override { estimate_.fill(0.); }

  void oplusImpl(const VectorX::MapType& update) override {
    estimate_ += update.head<kDimension>();
  }

  bool setEstimateDataImpl(const number_t* est) override {
    Eigen::Map<const Vector3> estMap(est);
    estimate_ = estMap;
    return true;
  }

  bool getEstimateData(number_t* est) const override {
    Eigen::Map<Vector3> estMap(est);
    estMap = estimate_;
    return true;
  }

  int estimateDimension() const override { return kDimension; }

  bool setMinimalEstimateDataImpl(const number_t* est) override {
    estimate_ = Eigen::Map<const Vector3>(est);
    return true;
  }

  bool getMinimalEstimateData(number_t* est) const override {
    Eigen::Map<Vector3> v(est);
    v = estimate_;
    return true;
  }

  int minimalEstimateDimension() const override { return kDimension; }
};

class G2O_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexPointXYZWriteGnuplotAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};

#ifdef G2O_HAVE_OPENGL
/**
 * \brief visualize a 3D point
 */
class VertexPointXYZDrawAction : public DrawAction {
 public:
  VertexPointXYZDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;

 protected:
  std::shared_ptr<FloatProperty> pointSize_;
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
};
#endif

}  // namespace g2o
#endif
