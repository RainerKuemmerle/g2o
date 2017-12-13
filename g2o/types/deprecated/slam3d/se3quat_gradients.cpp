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

#include "g2o/types/slam3d/se3quat.h"

namespace g2o {
namespace deprecated {

  /** conversion code from Euler angles */
Quaternion euler_to_quat(double yaw, double pitch, double roll)
{
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
  return Quaternion(w,x,y,z);
}

void quat_to_euler(const Quaternion& q, double& yaw, double& pitch, double& roll)
{
  const double& q0 = q.w();
  const double& q1 = q.x();
  const double& q2 = q.y();
  const double& q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

void jac_quat3_euler3(Eigen::Matrix<number_t, 6, 6>& J, const SE3Quat& t)
{
  const Vector3& tr0 = t.translation();
  const Quaternion& q0 = t.rotation();

  double delta=1e-6;
  double idelta= 1. / (2. * delta);

  for (int i=0; i<6; i++){
    SE3Quat ta, tb;
    if (i<3){
      Vector3 tra=tr0;
      Vector3 trb=tr0;
      tra[i] -= delta;
      trb[i] += delta;
      ta = SE3Quat(q0, tra); 
      tb = SE3Quat(q0, trb); 
    } else {
      Quaternion qa=q0;
      Quaternion qb=q0;
      if (i == 3) {
        qa.x() -= delta;
        qb.x() += delta;
      }
      else if (i == 4) {
        qa.y() -= delta;
        qb.y() += delta;
      }
      else if (i == 5) {
        qa.z() -= delta;
        qb.z() += delta;
      }
      qa.normalize();
      qb.normalize();
      ta = SE3Quat(qa, tr0); 
      tb = SE3Quat(qb, tr0); 
    }

    Vector3 dtr = (tb.translation() - ta.translation())*idelta;
    Vector3 taAngles, tbAngles;
    quat_to_euler(ta.rotation(), taAngles(2), taAngles(1), taAngles(0));
    quat_to_euler(tb.rotation(), tbAngles(2), tbAngles(1), tbAngles(0));
    Vector3 da = (tbAngles - taAngles) * idelta; //TODO wraparounds not handled

    for (int j=0; j<6; j++){
      if (j<3){
        J(j, i) = dtr(j);
      } else {
        J(j, i) = da(j-3);
      }
    }
  }
}

} // end namespace
} // end namespace
