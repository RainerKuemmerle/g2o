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

#ifndef G2O_CAMERA_PARAMETERS_H_
#define G2O_CAMERA_PARAMETERS_H_

#include <memory>
#include <utility>

#include "g2o/config.h"
#include "g2o/core/cache.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/parameter.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/property.h"
#include "g2o_types_slam3d_api.h"
#include "type_traits_isometry3.h"

namespace g2o {

class CameraWithOffset {
 public:
  CameraWithOffset() : offset_(Isometry3::Identity()) {
    setKcam(1, 1, 0.5, 0.5);
  }

  CameraWithOffset(Isometry3 offset, double fx, double fy, double cx, double cy)
      : offset_(std::move(offset)) {
    setKcam(fx, fy, cx, cy);
  }

  [[nodiscard]] const Matrix3& Kcam() const { return Kcam_; }
  void setKcam(double fx, double fy, double cx, double cy) {
    Kcam_.setZero();
    Kcam_(0, 0) = fx;
    Kcam_(1, 1) = fy;
    Kcam_(0, 2) = cx;
    Kcam_(1, 2) = cy;
    Kcam_(2, 2) = 1.0;
    update();
  }

  [[nodiscard]] const Isometry3& offset() const { return offset_; }
  [[nodiscard]] Isometry3& offset() { return offset_; }
  void setOffset(const Isometry3& offset) {
    offset_ = offset;
    update();
  }

  [[nodiscard]] const Matrix3& invKcam() const { return invKcam_; }
  [[nodiscard]] const Matrix3& KcamInverseOffsetR() const {
    return Kcam_inverseOffsetR_;
  }

 protected:
  virtual void update() {
    invKcam_ = Kcam_.inverse();
    Kcam_inverseOffsetR_ = Kcam_ * offset_.inverse().rotation();
  }

  Isometry3 offset_;
  Matrix3 Kcam_;
  Matrix3 invKcam_;
  Matrix3 Kcam_inverseOffsetR_;
};

template <>
struct TypeTraits<CameraWithOffset> {
  enum {  // NOLINT
    kVectorDimension = 11,
    kMinimalVectorDimension = 11,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = CameraWithOffset;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType result;
    result << TypeTraits<Isometry3>::toVector(t.offset()), t.Kcam()(0, 0),
        t.Kcam()(1, 1), t.Kcam()(0, 2), t.Kcam()(1, 2);
    return result;
  }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = toVector(t);
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return toVector(t);
  }
  static void toMinimalData(const Type& t, double* data) { toData(t, data); }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    Isometry3 offset = TypeTraits<Isometry3>::fromVector(v.template head<7>());
    Vector4 cam_params = v.template tail<4>();
    return Type(offset, cam_params(0), cam_params(1), cam_params(2),
                cam_params(3));
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return fromVector(v);
  }
};

/**
 * \brief parameters for a camera
 */
class G2O_TYPES_SLAM3D_API ParameterCamera
    : public BaseParameter<CameraWithOffset> {
 public:
  ParameterCamera() = default;

  void update() override;
};

class G2O_TYPES_SLAM3D_API CacheCamera : public Cache {
 public:
  using ParameterType = ParameterCamera;

  //! parameters of the camera
  [[nodiscard]] const ParameterType* camParams() const {
    return static_cast<ParameterType*>(parameters_[0].get());
  }

  [[nodiscard]] const Isometry3& w2n() const { return w2n_; }
  [[nodiscard]] const Isometry3& n2w() const { return n2w_; }
  [[nodiscard]] const Isometry3& w2l() const { return w2l_; }

  //! return the world to image transform
  [[nodiscard]] const Affine3& w2i() const { return w2i_; }

 protected:
  void updateImpl() override;
  Isometry3 w2n_;
  Isometry3 n2w_;
  Isometry3 w2l_;
  Affine3 w2i_;  ///< world to image transform
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM3D_API CacheCameraDrawAction : public DrawAction {
 public:
  CacheCameraDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  HyperGraphElementAction::Parameters& params_) override;

 protected:
  DrawAction::Parameters* refreshPropertyPtrs(
      HyperGraphElementAction::Parameters& params_) override;
  std::shared_ptr<FloatProperty> cameraZ_, cameraSide_;
};
#endif

}  // namespace g2o

#endif
