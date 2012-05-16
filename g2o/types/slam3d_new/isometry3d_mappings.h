// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_ISOMATRY3D_MAPPINGS_NEW_H_
#define G2O_ISOMATRY3D_MAPPINGS_NEW_H_

#include "g2o_types_slam3d_new_api.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Slam3dNew {
  using namespace Eigen;
  
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
  
  inline Quaterniond G2O_TYPES_SLAM3D_NEW_API normalized(const Quaterniond& q);
  inline Quaterniond& G2O_TYPES_SLAM3D_NEW_API normalize(Quaterniond& q);

  // functions to handle the rotation part
  inline Vector3d G2O_TYPES_SLAM3D_NEW_API toEuler(const Eigen::Matrix3d& R);
  inline Matrix3d G2O_TYPES_SLAM3D_NEW_API fromEuler(const Vector3d& v);
  inline Vector3d G2O_TYPES_SLAM3D_NEW_API toCompactQuaternion(const Eigen::Matrix3d& R);
  inline Matrix3d G2O_TYPES_SLAM3D_NEW_API fromCompactQuaternion(const Vector3d& v);

  
  // functions to handle the toVector of the whole transformations
  inline Vector6d G2O_TYPES_SLAM3D_NEW_API toVectorMQT(const Isometry3d& t);
  inline Vector6d G2O_TYPES_SLAM3D_NEW_API toVectorET(const Isometry3d& t);
  inline Vector7d G2O_TYPES_SLAM3D_NEW_API toVectorQT(const Isometry3d& t);
  
  inline Isometry3d G2O_TYPES_SLAM3D_NEW_API fromVectorMQT(const Vector6d& v);
  inline Isometry3d G2O_TYPES_SLAM3D_NEW_API fromVectorET(const Vector6d& v);
  inline Isometry3d G2O_TYPES_SLAM3D_NEW_API fromVectorQT(const Vector7d& v);
  
}

#include "isometry3d_mappings.cpp"
#endif
