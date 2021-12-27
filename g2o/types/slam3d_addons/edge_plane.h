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

#ifndef G2O_EDGE_PLANE3D_H
#define G2O_EDGE_PLANE3D_H

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam3d_addons_api.h"
#include "vertex_plane.h"

namespace g2o {

class G2O_TYPES_SLAM3D_ADDONS_API EdgePlane
    : public BaseBinaryEdge<4, Vector4, VertexPlane, VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlane();

  void computeError() override {
    auto* v1 = vertexXnRaw<0>();
    auto* v2 = vertexXnRaw<1>();
    error_ =
        (v2->estimate().toVector() - v1->estimate().toVector()) - measurement_;
  }
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setMeasurement(const Vector4& m) override { measurement_ = m; }

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector4> m(d);
    measurement_ = m;
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector4> m(d);
    m = measurement_;
    return true;
  }

  int measurementDimension() const override { return 4; }

  bool setMeasurementFromState() override {
    auto* v1 = vertexXnRaw<0>();
    auto* v2 = vertexXnRaw<1>();
    measurement_ = (v2->estimate().toVector()) - v1->estimate().toVector();

    return true;
  }

#if 0
    virtual void linearizeOplus();
#endif
};

}  // namespace g2o

#endif
