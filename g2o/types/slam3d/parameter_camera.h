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

#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam3d_api.h"
#include "parameter_se3_offset.h"

namespace g2o {
/**
 * \brief parameters for a camera
 */
class G2O_TYPES_SLAM3D_API ParameterCamera : public ParameterSE3Offset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ParameterCamera();
  void setKcam(number_t fx, number_t fy, number_t cx, number_t cy);
  void setOffset(const Isometry3& offset_ = Isometry3::Identity());

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  const Matrix3& Kcam() const { return Kcam_; }
  const Matrix3& invKcam() const { return invKcam_; }
  const Matrix3& Kcam_inverseOffsetR() const { return Kcam_inverseOffsetR_; }

 protected:
  Matrix3 Kcam_;
  Matrix3 invKcam_;
  Matrix3 Kcam_inverseOffsetR_;
};

class G2O_TYPES_SLAM3D_API CacheCamera : public CacheSE3Offset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using ParameterType = ParameterCamera;

  //! parameters of the camera
  std::shared_ptr<ParameterType> camParams() const {
    return std::static_pointer_cast<ParameterType>(parameters_[0]);
  }

  //! return the world to image transform
  const Affine3& w2i() const { return w2i_; }

 protected:
  void updateImpl() override;
  Affine3 w2i_;  ///< world to image transform
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM3D_API CacheCameraDrawAction : public DrawAction {
 public:
  CacheCameraDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<FloatProperty> cameraZ_, cameraSide_;
};
#endif

}  // namespace g2o

#endif
