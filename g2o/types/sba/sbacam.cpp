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

#include "sbacam.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"

namespace g2o {

// initialize an object
SBACam::SBACam() {
  setKcam(1, 1, cst(0.5), cst(0.5), 0);  // unit image projection
}

// set the object pose
SBACam::SBACam(const Quaternion& r_, const Vector3& t_) : SE3Quat(r_, t_) {
  Kcam.setZero();
  setTransform();
  setProjection();
  setDr();
}

SBACam::SBACam(const SE3Quat& p) : SE3Quat(p) {
  Kcam.setZero();
  setTransform();
  setProjection();
  setDr();
}

// update from the linear solution
// defined in se3quat
void SBACam::update(const Vector6& update) {
  // position update
  _t += update.head(3);
  // small quaternion update
  Quaternion qr;
  qr.vec() = update.segment<3>(3);
  qr.w() = sqrt(cst(1.0) - qr.vec().squaredNorm());  // should always be positive
  _r *= qr;                                          // post-multiply
  _r.normalize();
  setTransform();
  setProjection();
  setDr();
}

// transforms
void SBACam::transformW2F(Eigen::Matrix<number_t, 3, 4>& m, const Vector3& trans,
                          const Quaternion& qrot) {
  m.block<3, 3>(0, 0) = qrot.toRotationMatrix().transpose();
  m.col(3).setZero();  // make sure there's no translation
  Vector4 tt;
  tt.head(3) = trans;
  tt[3] = 1.0;
  m.col(3) = -m * tt;
}

void SBACam::transformF2W(Eigen::Matrix<number_t, 3, 4>& m, const Vector3& trans,
                          const Quaternion& qrot) {
  m.block<3, 3>(0, 0) = qrot.toRotationMatrix();
  m.col(3) = trans;
}

// set up camera matrix
void SBACam::setKcam(number_t fx, number_t fy, number_t cx, number_t cy, number_t tx) {
  Kcam.setZero();
  Kcam(0, 0) = fx;
  Kcam(1, 1) = fy;
  Kcam(0, 2) = cx;
  Kcam(1, 2) = cy;
  Kcam(2, 2) = cst(1.0);
  baseline = tx;
  setTransform();
  setProjection();
  setDr();
}

// sets angle derivatives
void SBACam::setDr() {
  // inefficient, just for testing
  // use simple multiplications and additions for production code in calculating dRdx,y,z
  Matrix3 dRidx, dRidy, dRidz;
  dRidx << cst(0.0), cst(0.0), cst(0.0), cst(0.0), cst(0.0), cst(2.0), cst(0.0), cst(-2.0),
      cst(0.0);
  dRidy << cst(0.0), cst(0.0), cst(-2.0), cst(0.0), cst(0.0), cst(0.0), cst(2.0), cst(0.0),
      cst(0.0);
  dRidz << cst(0.0), cst(2.0), cst(0.0), cst(-2.0), cst(0.0), cst(0.0), cst(0.0), cst(0.0),
      cst(0.0);

  // for dS'*R', with dS the incremental change
  dRdx = dRidx * w2n.block<3, 3>(0, 0);
  dRdy = dRidy * w2n.block<3, 3>(0, 0);
  dRdz = dRidz * w2n.block<3, 3>(0, 0);
}

// human-readable SBACam object
std::ostream& operator<<(std::ostream& out_str, const SBACam& cam) {
  out_str << cam.translation().transpose() << std::endl;
  out_str << cam.rotation().coeffs().transpose() << std::endl << std::endl;
  out_str << cam.Kcam << std::endl << std::endl;
  out_str << cam.w2n << std::endl << std::endl;
  out_str << cam.w2i << std::endl << std::endl;
  return out_str;
}

}  // namespace g2o
