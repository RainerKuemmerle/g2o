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

#ifndef G2O_TYPES_ICP
#define G2O_TYPES_ICP

#include "g2o/core/eigen_types.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

#define GICP_ANALYTIC_JACOBIANS
// #define SCAM_ANALYTIC_JACOBIANS

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "edge_gicp.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_icp_api.h"

namespace g2o {

// 3D rigid constraint
//    3 values for position wrt frame
//    3 values for normal wrt frame, not used here
// first two args are the measurement type, second two the connection classes
class G2O_TYPES_ICP_API EdgeVVGicp
    : public BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3> {
 public:
  EdgeVVGicp() = default;

  // switch to go between point-plane and plane-plane
  bool pl_pl = false;
  Matrix3 cov0, cov1;

  // return the error estimate as a 3-vector
  void computeError() override;

  // try analytic jacobians
#ifdef GICP_ANALYTIC_JACOBIANS
  void linearizeOplus() override;
#endif

 protected:
  // global derivative matrices
  static const Matrix3 kDRidx;
  static const Matrix3 kDRidy;
  static const Matrix3 kDRidz;  // differential quat matrices
};

/**
 * \brief Stereo camera vertex, derived from SE3 class.
 * Note that we use the actual pose of the vertex as its parameterization,
 * rather than the transform from RW to camera coords. Uses static vars for
 * camera params, so there is a single camera setup.
 */
class G2O_TYPES_ICP_API VertexSCam : public VertexSE3 {
 public:
  // capture the update function to reset aux transforms
  void oplusImpl(const VectorX::MapType& update) override;

  // camera matrix and stereo baseline
  static Matrix3 kcam_;
  static double baseline_;

  // transformations
  Eigen::Matrix<double, 3, 4, Eigen::ColMajor>
      w2n;  // transform from world to node coordinates
  Eigen::Matrix<double, 3, 4, Eigen::ColMajor>
      w2i;  // transform from world to image coordinates

  // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
  // calculating Jacobian wrt pose of a projection.
  Matrix3 dRdx, dRdy, dRdz;

  // transforms
  static void transformW2F(Eigen::Matrix<double, 3, 4, Eigen::ColMajor>& m,
                           const Vector3& trans, const Quaternion& qrot);

  static void transformF2W(Eigen::Matrix<double, 3, 4, Eigen::ColMajor>& m,
                           const Vector3& trans, const Quaternion& qrot);

  // set up camera matrix
  static void setKcam(double fx, double fy, double cx, double cy, double tx);

  // set transform from world to cam coords
  void setTransform();

  // Set up world-to-image projection matrix (w2i), assumes camera parameters
  // are filled.
  void setProjection();

  // sets angle derivatives
  void setDr();

  // set all aux transforms
  void setAll();

  // calculate stereo projection
  void mapPoint(Vector3& res, const Vector3& pt3) const;

 protected:
  static const Matrix3 kDRidx;
  static const Matrix3 kDRidy;
  static const Matrix3 kDRidz;
};

/**
 * \brief Point vertex, XYZ, is in types_sba
 */

// stereo projection
// first two args are the measurement type, second two the connection classes
class G2O_TYPES_ICP_API EdgeXyzVsc
    : public BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSCam> {
 public:
  // return the error estimate as a 3-vector
  void computeError() override;

#ifdef SCAM_ANALYTIC_JACOBIANS
  // jacobian
  virtual void linearizeOplus();
#endif
};

}  // namespace g2o

#endif  // TYPES_ICP
