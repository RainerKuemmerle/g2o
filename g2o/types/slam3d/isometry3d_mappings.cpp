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

namespace g2o {
  namespace internal {

    Eigen::Quaterniond normalized(const Eigen::Quaterniond& q) {
      Eigen::Quaterniond q2(q);
      normalize(q2);
      return q2;
    }

    Eigen::Quaterniond& normalize(Eigen::Quaterniond& q){
      q.normalize();
      if (q.w()<0) {
        q.coeffs() *= -1;
      }
      return q;
    }

    // functions to handle the rotation part
    Vector3D toEuler(const Matrix3D& R) {
      Eigen::Quaterniond q(R);
      const double& q0 = q.w();
      const double& q1 = q.x();
      const double& q2 = q.y();
      const double& q3 = q.z();
      double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
      double pitch = asin(2*(q0*q2-q3*q1));
      double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
      return Vector3D(roll, pitch, yaw);
    }

    Matrix3D fromEuler(const Vector3D& v) {
      //UNOPTIMIZED
      double roll  = v[0];
      double pitch = v[1];
      double yaw   = v[2];
      double sy = sin(yaw*0.5);
      double cy = cos(yaw*0.5);
      double sp = sin(pitch*0.5);
      double cp = cos(pitch*0.5);
      double sr = sin(roll*0.5);
      double cr = cos(roll*0.5);
      double w = cr*cp*cy + sr*sp*sy;
      double x = sr*cp*cy - cr*sp*sy;
      double y = cr*sp*cy + sr*cp*sy;
      double z = cr*cp*sy - sr*sp*cy;
      return Eigen::Quaterniond(w,x,y,z).toRotationMatrix();
    }

    Vector3D toCompactQuaternion(const Matrix3D& R) {
      Eigen::Quaterniond q(R);
      normalize(q);
      // return (x,y,z) of the quaternion
      return q.coeffs().head<3>();
    }

    Matrix3D fromCompactQuaternion(const Vector3D& v) {
      double w = 1-v.squaredNorm();
      if (w<0)
        return Matrix3D::Identity();
      else
        w=sqrt(w);
      return Eigen::Quaterniond(w, v[0], v[1], v[2]).toRotationMatrix();
    }

    // functions to handle the toVector of the whole transformations
    Vector6d toVectorMQT(const Isometry3D& t) {
      Vector6d v;
      v.block<3,1>(3,0) = toCompactQuaternion(extractRotation(t));
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Vector6d toVectorET(const Isometry3D& t) {
      Vector6d v;
      v.block<3,1>(3,0)=toEuler(extractRotation(t));
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Vector7d toVectorQT(const Isometry3D& t){
      Eigen::Quaterniond q(extractRotation(t));
      q.normalize();
      Vector7d v;
      v[3] = q.x(); v[4] = q.y(); v[5] = q.z(); v[6] = q.w();
      v.block<3,1>(0,0) = t.translation();
      return v;
    }

    Isometry3D fromVectorMQT(const Vector6d& v){
      Isometry3D t;
      t = fromCompactQuaternion(v.block<3,1>(3,0));
      t.translation() = v.block<3,1>(0,0);
      return t;
    }

    Isometry3D fromVectorET(const Vector6d& v) {
      Isometry3D t;
      t = fromEuler(v.block<3,1>(3,0));
      t.translation() = v.block<3,1>(0,0);
      return t;
    }

    Isometry3D fromVectorQT(const Vector7d& v) {
      Isometry3D t;
      t=Eigen::Quaterniond(v[6], v[3], v[4], v[5]).toRotationMatrix();
      t.translation() = v.head<3>();
      return t;
    }

    SE3Quat toSE3Quat(const Isometry3D& t)
    {
      SE3Quat result(t.matrix().topLeftCorner<3,3>(), t.translation());
      return result;
    }

    Isometry3D fromSE3Quat(const SE3Quat& t)
    {
      Isometry3D result = (Isometry3D) t.rotation();
      result.translation() = t.translation();
      return result;
    }

  } // end namespace internal
} // end namespace g2o
