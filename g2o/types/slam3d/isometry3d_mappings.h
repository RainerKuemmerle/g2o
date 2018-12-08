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

namespace g2o {

  /**
   * internal functions used inside g2o.
   * Those functions may disappear or change their meaning without further
   * notification
   */
  namespace internal {

    /**
     * extract the rotation matrix from an Isometry3 matrix. Eigen itself
     * performs an SVD decomposition to recover the nearest orthogonal matrix,
     * since its rotation() function also handles a scaling matrix.  An Isometry3 does
     * not have a scaling portion and we assume that the Isometry3 is
     * numerically stable while we compute the error and the Jacobians.  Hence,
     * we directly extract the rotation block out of the full matrix.
     *
     * Note, we could also call .linear() on the Isometry3. However, I dislike
     * the name of that function a bit.
     */
    inline Isometry3::ConstLinearPart extractRotation(const Isometry3& A)
    {
      return A.matrix().topLeftCorner<3,3>();
    }

    /**
     * computes the nearest orthogonal matrix of a rotation matrix which might
     * be affected by numerical inaccuracies. We periodically call this function
     * after performinag a large number of updates on vertices.
     * This function computes an SVD to reconstruct the nearest orthogonal matrix.
     */
    template <typename Derived>
    void nearestOrthogonalMatrix(const Eigen::MatrixBase<Derived>& R)
    {
      Eigen::JacobiSVD<Matrix3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
      number_t det = (svd.matrixU() * svd.matrixV().adjoint()).determinant();
      Matrix3 scaledU(svd.matrixU());
      scaledU.col(0) /= det;
      const_cast<Eigen::MatrixBase<Derived>&>(R) = scaledU * svd.matrixV().transpose();
    }

    /**
     * compute a fast approximation for the nearest orthogonal rotation matrix.
     * The function computes the residual E = RR^T - I which is then used as follows:
     * R := R - 1/2 R E
     */
    template <typename Derived>
    void approximateNearestOrthogonalMatrix(const Eigen::MatrixBase<Derived>& R)
    {
      Matrix3 E = R.transpose() * R;
      E.diagonal().array() -= 1;
      const_cast<Eigen::MatrixBase<Derived>&>(R) -= 0.5 * R * E;
    }

    /**
     * normalize the quaternion, such that ||q|| == 1 and q.w() > 0
     */
    G2O_TYPES_SLAM3D_API Quaternion normalized(const Quaternion& q);
    /**
     * as above, but in-place
     */
    G2O_TYPES_SLAM3D_API Quaternion& normalize(Quaternion& q);

    // functions to handle the rotation part
    /**
     * Rotation matrix -> Euler angles (roll, pitch, yaw)
     */
    G2O_TYPES_SLAM3D_API Vector3 toEuler(const Matrix3& R);
    /**
     * Euler angles (roll, pitch, yaw) -> Rotation matrix
     */
    G2O_TYPES_SLAM3D_API Matrix3 fromEuler(const Vector3& v);
    /**
     * Rotation matrix -> (qx qy, qz)
     */
    G2O_TYPES_SLAM3D_API Vector3 toCompactQuaternion(const Matrix3& R);
    /**
     * (qx qy, qz) -> Rotation matrix, whereas (qx, qy, qz) are assumed to be
     * part of a quaternion which was normalized with the function above.
     */
    G2O_TYPES_SLAM3D_API Matrix3 fromCompactQuaternion(const Vector3& v);

    // functions to handle the toVector of the whole transformations
    /**
     * Isometry3 -> (x, y, z, qx, qy, qz)
     */
    G2O_TYPES_SLAM3D_API Vector6 toVectorMQT(const Isometry3& t);
    /**
     * Isometry3 -> (x, y, z, roll, pitch, yaw)
     */
    G2O_TYPES_SLAM3D_API Vector6 toVectorET(const Isometry3& t);
    /**
     * Isometry3 -> (x, y, z, qx, qy, qz, qw)
     */
    G2O_TYPES_SLAM3D_API Vector7 toVectorQT(const Isometry3& t);

    /**
     * (x, y, z, qx, qy, qz) -> Isometry3
     */
    G2O_TYPES_SLAM3D_API Isometry3 fromVectorMQT(const Vector6& v);
    /**
     * (x, y, z, roll, pitch, yaw) -> Isometry3
     */
    G2O_TYPES_SLAM3D_API Isometry3 fromVectorET(const Vector6& v);
    /**
     * (x, y, z, qx, qy, qz, qw) -> Isometry3
     */
    G2O_TYPES_SLAM3D_API Isometry3 fromVectorQT(const Vector7& v);

    /**
     * convert an Isometry3 to the old SE3Quat class
     */
    G2O_TYPES_SLAM3D_API SE3Quat toSE3Quat(const Isometry3& t);
    /**
     * convert from an old SE3Quat into Isometry3
     */
    G2O_TYPES_SLAM3D_API Isometry3 fromSE3Quat(const SE3Quat& t);

  } // end namespace internal
} // end namespace g2o

#endif
