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

  typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;
  typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;

  /**
   * internal functions used inside g2o.
   * Those functions may disappear or change their meaning without further
   * notification
   */
  namespace internal {

    /**
     * extract the rotation matrix from an Isometry3D matrix. Eigen itself
     * performs an SVD decomposition to recover the nearest orthogonal matrix,
     * since its rotation() function also handles a scaling matrix.  An Isometry3D does
     * not have a scaling portion and we assume that the Isometry3D is
     * numerically stable while we compute the error and the Jacobians.  Hence,
     * we directly extract the rotation block out of the full matrix.
     *
     * Note, we could also call .linear() on the Isometry3D. However, I dislike
     * the name of that function a bit.
     */
    inline Isometry3D::ConstLinearPart extractRotation(const Isometry3D& A)
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
      Eigen::JacobiSVD<Matrix3D> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
      double det = (svd.matrixU() * svd.matrixV().adjoint()).determinant();
      Matrix3D scaledU(svd.matrixU());
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
      Matrix3D E = R.transpose() * R;
      E.diagonal().array() -= 1;
      const_cast<Eigen::MatrixBase<Derived>&>(R) -= 0.5 * R * E;
    }

    /**
     * normalize the quaternion, such that ||q|| == 1 and q.w() > 0
     */
    G2O_TYPES_SLAM3D_API Eigen::Quaterniond normalized(const Eigen::Quaterniond& q);
    /**
     * as above, but in-place
     */
    G2O_TYPES_SLAM3D_API Eigen::Quaterniond& normalize(Eigen::Quaterniond& q);

    // functions to handle the rotation part
    /**
     * Rotation matrix -> Euler angles (roll, pitch, yaw)
     */
    G2O_TYPES_SLAM3D_API Vector3D toEuler(const Matrix3D& R);
    /**
     * Euler angles (roll, pitch, yaw) -> Rotation matrix
     */
    G2O_TYPES_SLAM3D_API Matrix3D fromEuler(const Vector3D& v);
    /**
     * Rotation matrix -> (qx qy, qz)
     */
    G2O_TYPES_SLAM3D_API Vector3D toCompactQuaternion(const Matrix3D& R);
    /**
     * (qx qy, qz) -> Rotation matrix, whereas (qx, qy, qz) are assumed to be
     * part of a quaternion which was normalized with the function above.
     */
    G2O_TYPES_SLAM3D_API Matrix3D fromCompactQuaternion(const Vector3D& v);

    // functions to handle the toVector of the whole transformations
    /**
     * Isometry3D -> (x, y, z, qx, qy, qz)
     */
    G2O_TYPES_SLAM3D_API Vector6d toVectorMQT(const Isometry3D& t);
    /**
     * Isometry3D -> (x, y, z, roll, pitch, yaw)
     */
    G2O_TYPES_SLAM3D_API Vector6d toVectorET(const Isometry3D& t);
    /**
     * Isometry3D -> (x, y, z, qx, qy, qz, qw)
     */
    G2O_TYPES_SLAM3D_API Vector7d toVectorQT(const Isometry3D& t);

    /**
     * (x, y, z, qx, qy, qz) -> Isometry3D
     */
    G2O_TYPES_SLAM3D_API Isometry3D fromVectorMQT(const Vector6d& v);
    /**
     * (x, y, z, roll, pitch, yaw) -> Isometry3D
     */
    G2O_TYPES_SLAM3D_API Isometry3D fromVectorET(const Vector6d& v);
    /**
     * (x, y, z, qx, qy, qz, qw) -> Isometry3D
     */
    G2O_TYPES_SLAM3D_API Isometry3D fromVectorQT(const Vector7d& v);

    /**
     * convert an Isometry3D to the old SE3Quat class
     */
    G2O_TYPES_SLAM3D_API SE3Quat toSE3Quat(const Isometry3D& t);
    /**
     * convert from an old SE3Quat into Isometry3D
     */
    G2O_TYPES_SLAM3D_API Isometry3D fromSE3Quat(const SE3Quat& t);

  } // end namespace internal
} // end namespace g2o

#endif
