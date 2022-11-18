// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_SBA_VERTEX_CAM_H
#define G2O_SBA_VERTEX_CAM_H

#include "g2o/core/base_vertex.h"
#include "g2o_types_sba_api.h"
#include "sbacam.h"

namespace g2o {

/**
 * \brief SBACam Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 * qw is assumed to be positive, otherwise there is an ambiguity in qx,qy,qz as
 * a rotation
 */
class G2O_TYPES_SBA_API VertexCam : public BaseVertex<6, SBACam> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setToOriginImpl() override { estimate_ = SBACam(); }

  virtual void setEstimate(const SBACam& cam) {
    BaseVertex<6, SBACam>::setEstimate(cam);
    estimate_.setTransform();
    estimate_.setProjection();
    estimate_.setDr();
  }

  void oplusImpl(const VectorX::MapType& update) override {
    estimate_.update(update.head<kDimension>());
    estimate_.setTransform();
    estimate_.setProjection();
    estimate_.setDr();
  }

  bool setEstimateDataImpl(const number_t* est) override {
    Eigen::Map<const Vector7> v(est);
    estimate_.fromVector(v);
    return true;
  }

  bool getEstimateData(number_t* est) const override {
    Eigen::Map<Vector7> v(est);
    v = estimate().toVector();
    return true;
  }

  int estimateDimension() const override { return 7; }

  bool setMinimalEstimateDataImpl(const number_t* est) override {
    Eigen::Map<const Vector6> v(est);
    estimate_.fromMinimalVector(v);
    return true;
  }

  bool getMinimalEstimateData(number_t* est) const override {
    Eigen::Map<Vector6> v(est);
    v = estimate_.toMinimalVector();
    return true;
  }

  int minimalEstimateDimension() const override { return 6; }
};
}  // namespace g2o

#endif
