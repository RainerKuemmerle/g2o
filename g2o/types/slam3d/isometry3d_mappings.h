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

#ifndef G2O_ISOMETRY3D_MAPPINGS_H_
#define G2O_ISOMETRY3D_MAPPINGS_H_

#include "g2o_types_slam3d_api.h"
#include "se3quat.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  using namespace Eigen;

  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;

  /**
   * internal functions used inside g2o.
   * Those functions may disappear or change their meaning without further
   * notification
   */
  namespace internal {

    /**
     * extract the rotation matrix from an Isometry3d matrix. Eigen itself
     * performs an SVD decomposition to recover the nearest orthogonal matrix,
     * since its function also handles a scaling matrix.  An Isometry3d does
     * not have a scaling portion and we assume that the Isometry3d is
     * numerically stable while we compute the error and the Jacobians.  Hence,
     * we directly extract the rotation block out of the full matrix.
     */
    inline Eigen::Matrix3d extractRotation(const Eigen::Isometry3d& A)
    {
      Matrix3d m = A.matrix().topLeftCorner<3,3>();
      return m;
    }

    /**
     * computes the nearest orthogonal matrix of a rotation matrix which might
     * be affected by numerical inaccuracies. We periodically call this function
     * after performinag a large number of updates on vertices.
     */
    Eigen::Matrix3d nearestOrthogonalMatrix(const Eigen::Matrix3d& R);

    /**
     * normalize the quaternion, such that ||q|| == 1 and q.w() > 0
     */
    Quaterniond G2O_TYPES_SLAM3D_API normalized(const Quaterniond& q);
    /**
     * as above, but in-place
     */
    Quaterniond& G2O_TYPES_SLAM3D_API normalize(Quaterniond& q);

    // functions to handle the rotation part
    /**
     * Rotation matrix -> Euler angles (roll, pitch, yaw)
     */
    Vector3d G2O_TYPES_SLAM3D_API toEuler(const Eigen::Matrix3d& R);
    /**
     * Euler angles (roll, pitch, yaw) -> Rotation matrix
     */
    Matrix3d G2O_TYPES_SLAM3D_API fromEuler(const Vector3d& v);
    /**
     * Rotation matrix -> (qx qy, qz)
     */
    Vector3d G2O_TYPES_SLAM3D_API toCompactQuaternion(const Eigen::Matrix3d& R);
    /**
     * (qx qy, qz) -> Rotation matrix, whereas (qx, qy, qz) are assumed to be
     * part of a quaternion which was normalized with the function above.
     */
    Matrix3d G2O_TYPES_SLAM3D_API fromCompactQuaternion(const Vector3d& v);

    // functions to handle the toVector of the whole transformations
    /**
     * Isometry3d -> (x, y, z, qx, qy, qz)
     */
    Vector6d G2O_TYPES_SLAM3D_API toVectorMQT(const Isometry3d& t);
    /**
     * Isometry3d -> (x, y, z, roll, pitch, yaw)
     */
    Vector6d G2O_TYPES_SLAM3D_API toVectorET(const Isometry3d& t);
    /**
     * Isometry3d -> (x, y, z, qx, qy, qz, qw)
     */
    Vector7d G2O_TYPES_SLAM3D_API toVectorQT(const Isometry3d& t);

    /**
     * (x, y, z, qx, qy, qz) -> Isometry3d
     */
    Isometry3d G2O_TYPES_SLAM3D_API fromVectorMQT(const Vector6d& v);
    /**
     * (x, y, z, roll, pitch, yaw) -> Isometry3d
     */
    Isometry3d G2O_TYPES_SLAM3D_API fromVectorET(const Vector6d& v);
    /**
     * (x, y, z, qx, qy, qz, qw) -> Isometry3d
     */
    Isometry3d G2O_TYPES_SLAM3D_API fromVectorQT(const Vector7d& v);

    /**
     * convert an Isometry3d to the old SE3Quat class
     */
    SE3Quat G2O_TYPES_SLAM3D_API toSE3Quat(const Isometry3d& t);
    /**
     * convert from an old SE3Quat into Isometry3d
     */
    Isometry3d G2O_TYPES_SLAM3D_API fromSE3Quat(const SE3Quat& t);

  } // end namespace internal
} // end namespace g2o

#endif
