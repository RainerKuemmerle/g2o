// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat, G. Grisetti, R. Kümmerle, W. Burgard
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

#include "isometry3d_mappings.h"
#include "g2o/stuff/misc.h"

namespace g2o {
  namespace internal {

    Quaternion normalized(const Quaternion& q) {
      Quaternion q2(q);
      normalize(q2);
      return q2;
    }

    Quaternion& normalize(Quaternion& q){
      q.normalize();
      if (q.w()<0) {
        q.coeffs() *= -1;
      }
      return q;
    }

    // functions to handle the rotation part
    Vector3 toEuler(const Matrix3& R) {
      Quaternion q(R);
      const number_t& q0 = q.w();
      const number_t& q1 = q.x();
      const number_t& q2 = q.y();
      const number_t& q3 = q.z();
      number_t roll = std::atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
      number_t pitch = std::asin(2*(q0*q2-q3*q1));
      number_t yaw = std::atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
      return Vector3(roll, pitch, yaw);
    }

    Matrix3 fromEuler(const Vector3& v) {
      //UNOPTIMIZED
      number_t roll  = v[0];
      number_t pitch = v[1];
      number_t yaw   = v[2];
      number_t sy = std::sin(yaw*cst(0.5));
      number_t cy = std::cos(yaw*cst(0.5));
      number_t sp = std::sin(pitch*cst(0.5));
      number_t cp = std::cos(pitch*cst(0.5));
      number_t sr = std::sin(roll*cst(0.5));
      number_t cr = std::cos(roll*cst(0.5));
      number_t w = cr*cp*cy + sr*sp*sy;
      number_t x = sr*cp*cy - cr*sp*sy;
      number_t y = cr*sp*cy + sr*cp*sy;
      number_t z = cr*cp*sy - sr*sp*cy;
      return Quaternion(w,x,y,z).toRotationMatrix();
    }

    Vector3 toCompactQuaternion(const Matrix3& R) {
      Quaternion q(R);
      normalize(q);
      // return (x,y,z) of the quaternion
      return q.coeffs().head<3>();
    }

    Matrix3 fromCompactQuaternion(const Vector3& v) {
      number_t w = 1-v.squaredNorm();
      if (w<0)
        return Matrix3::Identity();
      else
        w=sqrt(w);
      return Quaternion(w, v[0], v[1], v[2]).toRotationMatrix();
    }

    // functions to handle the toVector of the whole transformations
    Vector6 toVectorMQT(const Isometry3& t) {
      Vector6 v;
      v.block<3,1>(3,0) = toCompactQuaternion(extractRotation(t));
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Vector6 toVectorET(const Isometry3& t) {
      Vector6 v;
      v.block<3,1>(3,0)=toEuler(extractRotation(t));
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Vector7 toVectorQT(const Isometry3& t){
      Quaternion q(extractRotation(t));
      q.normalize();
      Vector7 v;
      v[3] = q.x(); v[4] = q.y(); v[5] = q.z(); v[6] = q.w();
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Isometry3 fromVectorMQT(const Vector6& v){
      Isometry3 t;
      t = fromCompactQuaternion(v.block<3,1>(3,0));
      t.translation() = v.block<3,1>(0,0);
      return t;
    }

    Isometry3 fromVectorET(const Vector6& v) {
      Isometry3 t;
      t = fromEuler(v.block<3,1>(3,0));
      t.translation() = v.block<3,1>(0,0);
      return t;
    }

    Isometry3 fromVectorQT(const Vector7& v) {
      Isometry3 t;
      t=Quaternion(v[6], v[3], v[4], v[5]).toRotationMatrix();
      t.translation() = v.head<3>();
      return t;
    }

    SE3Quat toSE3Quat(const Isometry3& t)
    {
      SE3Quat result(t.matrix().topLeftCorner<3,3>(), t.translation());
      return result;
    }

    Isometry3 fromSE3Quat(const SE3Quat& t)
    {
      Isometry3 result = (Isometry3) t.rotation();
      result.translation() = t.translation();
      return result;
    }

  } // end namespace internal
} // end namespace g2o
