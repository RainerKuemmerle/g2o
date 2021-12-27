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

///
/// Basic types definitions for SBA translation to HChol
///
/// Camera nodes use camera pose in real world
///   v3 position
///   normalized quaternion rotation
///
/// Point nodes:
///   v3 position
///

#ifndef G2O_SBACam_H
#define G2O_SBACam_H

#include "g2o/types/slam3d/se3quat.h"
#include "g2o_types_sba_api.h"

namespace g2o {

class G2O_TYPES_SBA_API SBACam : public SE3Quat {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  // camera matrix and stereo baseline
  Matrix3 Kcam;
  number_t baseline;

  // transformations
  Eigen::Matrix<number_t, 3, 4>
      w2n;  // transform from world to node coordinates
  Eigen::Matrix<number_t, 3, 4>
      w2i;  // transform from world to image coordinates

  // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
  // calculating Jacobian wrt pose of a projection.
  Matrix3 dRdx, dRdy, dRdz;

  // initialize an object
  SBACam();

  // set the object pose
  SBACam(const Quaternion& r_, const Vector3& t_);

  SBACam(const SE3Quat& p);

  // update from the linear solution
  // defined in se3quat
  void update(const Vector6& update);

  // transforms
  static void transformW2F(Eigen::Matrix<number_t, 3, 4>& m,
                           const Vector3& trans, const Quaternion& qrot);

  static void transformF2W(Eigen::Matrix<number_t, 3, 4>& m,
                           const Vector3& trans, const Quaternion& qrot);

  // set up camera matrix
  void setKcam(number_t fx, number_t fy, number_t cx, number_t cy, number_t tx);

  // set transform from world to cam coords
  void setTransform() { transformW2F(w2n, _t, _r); }

  // Set up world-to-image projection matrix (w2i), assumes camera parameters
  // are filled.
  void setProjection() { w2i = Kcam * w2n; }

  // sets angle derivatives
  void setDr();
};

// human-readable SBACam object
std::ostream& operator<<(std::ostream& out_str, const SBACam& cam);

}  // namespace g2o

#endif  // SBACam
