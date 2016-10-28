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

#ifndef G2O_LINE3D_H_
#define G2O_LINE3D_H_

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o_types_slam3d_addons_api.h"
#include "g2o/stuff/misc.h"

namespace g2o {
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 4> Matrix6x4d;
  
  struct OrthonormalLine3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix2d W;
    Eigen::Matrix3d U;

    OrthonormalLine3D() {
      W = Eigen::Matrix2d::Identity();
      U = Eigen::Matrix3d::Identity();
    }
  };
  typedef struct OrthonormalLine3D OrthonormalLine3D;
  
  class G2O_TYPES_SLAM3D_ADDONS_API Line3D : public Vector6d {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    G2O_TYPES_SLAM3D_ADDONS_API friend Line3D operator*(const Eigen::Isometry3d& t,
    							const Line3D& line);

    Line3D() {
      *this << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    }

    G2O_TYPES_SLAM3D_ADDONS_API Line3D(const Vector6d& v) {
      (Vector6d&)*this = v;
    }

    G2O_TYPES_SLAM3D_ADDONS_API Vector6d toCartesian() const;

    G2O_TYPES_SLAM3D_ADDONS_API inline Eigen::Vector3d w() const {
      return head<3>();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Eigen::Vector3d d() const {
      return tail<3>();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void setW(const Eigen::Vector3d& w_) {
      head<3>() = w_;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void setD(const Eigen::Vector3d& d_) {
      tail<3>() = d_;
    }

    G2O_TYPES_SLAM3D_ADDONS_API static inline Line3D fromCartesian(const Vector6d& cart) {
      Line3D l;
      Eigen::Vector3d _d = cart.tail<3>() * 1.0/cart.tail<3>().norm();
      Eigen::Vector3d _p = cart.head<3>();
      _p -= _d*(_d.dot(_p));
      l.setW(_p.cross(_p+_d));
      l.setD(_d);
      return l;
    }

    G2O_TYPES_SLAM3D_ADDONS_API static inline Line3D fromOrthonormal(const OrthonormalLine3D& ortho) {
      /* mline_t *l = mline_create(); */
      /* l->moment->data[0] = MATD_EL(U, 0, 0) * MATD_EL(W, 0, 0); */
      /* l->moment->data[1] = MATD_EL(U, 1, 0) * MATD_EL(W, 0, 0); */
      /* l->moment->data[2] = MATD_EL(U, 2, 0) * MATD_EL(W, 0, 0); */
      /* l->direction->data[0] = MATD_EL(U, 0, 1) * MATD_EL(W, 1, 0); */
      /* l->direction->data[1] = MATD_EL(U, 1, 1) * MATD_EL(W, 1, 0); */
      /* l->direction->data[2] = MATD_EL(U, 2, 1) * MATD_EL(W, 1, 0); */
      /* mline_normalize(l); */
      /* return l; */

      Eigen::Vector3d w;
      w.x() = ortho.U(0, 0) * ortho.W(0, 0);
      w.y() = ortho.U(1, 0) * ortho.W(0, 0);
      w.z() = ortho.U(2, 0) * ortho.W(0, 0);

      Eigen::Vector3d d;
      d.x() = ortho.U(0, 1) * ortho.W(1, 0);
      d.y() = ortho.U(1, 1) * ortho.W(1, 0);
      d.z() = ortho.U(2, 1) * ortho.W(1, 0);

      Line3D l;
      l.setW(w);
      l.setD(d);
      l.normalize();
      
      return l;
    }

    G2O_TYPES_SLAM3D_ADDONS_API static inline OrthonormalLine3D toOrthonormal(const Line3D& line) {
      /* double dm = matd_vec_mag(l->direction); */
      /* double mm = matd_vec_mag(l->moment); */

      /* // W */
      /* double wn = 1.0 / sqrt(mm * mm + dm * dm); */
      /* double w[4] = {  */
      /* 	mm * wn, -dm * wn,  */
      /* 	dm * wn, mm * wn  */
      /* }; */
      /* *W = matd_create_data(2, 2, w);  */

      /* // U */
      /* double mn = 1.0 / mm; */
      /* double dn = 1.0 / dm; */
      /* double mdcross[3]; */
      /* doubles_cross_product(l->moment->data, l->direction->data, mdcross); */
      /* double mdcrossn = 1.0 / sqrt(mdcross[0]*mdcross[0] +  */
      /* 				   mdcross[1]*mdcross[1] +  */
      /* 				   mdcross[2]*mdcross[2]); */
      /* double u[9] = { */
      /* 	l->moment->data[0] * mn, l->direction->data[0] * dn, mdcross[0] * mdcrossn, */
      /* 	l->moment->data[1] * mn, l->direction->data[1] * dn, mdcross[1] * mdcrossn, */
      /* 	l->moment->data[2] * mn, l->direction->data[2] * dn, mdcross[2] * mdcrossn */
      /* }; */
      /* *U = matd_create_data(3, 3, u); */

      OrthonormalLine3D ortho;

      Eigen::Vector2d mags;
      mags << line.d().norm(), line.w().norm();

      // W
      double wn = 1.0 / mags.norm();
      ortho.W <<
      	mags.y() * wn, -mags.x() * wn,
      	mags.x() * wn,  mags.y() * wn;

      // U
      double mn = 1.0 / mags.y();
      double dn = 1.0 / mags.x();
      Eigen::Vector3d mdcross;
      mdcross = line.w().cross(line.d());
      double mdcrossn = 1.0 / mdcross.norm();
      ortho.W <<
      	line.w().x() * mn, line.d().x() * dn, mdcross.x() * mdcrossn,
      	line.w().y() * mn, line.d().y() * dn, mdcross.y() * mdcrossn,
      	line.w().z() * mn, line.d().z() * dn, mdcross.z() * mdcrossn;

      return ortho;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void normalize() {
      double n = 1.0/d().norm();
      (*this)*=n;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Line3D normalized() const {
      return Line3D((Vector6d)(*this)*(1.0/d().norm()));
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void oplus(const Eigen::Vector4d& v){
      /* matd_t *U, *W; */
      /* mline_to_orthonormal(l, &U, &W); */
    
      /* matd_t *R_33 = matd_create(3, 3); */
      /* double q[4];   */
      /* doubles_rpy_to_quat(v, q);   */
      /* doubles_quat_normalize_inplace(q); */
      /* doubles_quat_to_rot_mat(q, R_33->data); */
      /* double r_22[4] = {  */
      /* 	cos(v[3]), -sin(v[3]), */
      /* 	sin(v[3]),  cos(v[3]) */
      /* }; */
      /* matd_t *R_22 = matd_create_data(2, 2, r_22); */
  
      /* matd_product(U, R_33, U); */
      /* matd_product(W, R_22, W); */

      /* mline_from_orthonormal_inplace(l, U, W); */

      // transform *this to orthonormal
      // apply v through rotation matrices
      // bring the result back to pluecker and normalize

      OrthonormalLine3D ortho_estimate = toOrthonormal(*this);
      OrthonormalLine3D ortho_update;
      ortho_update.W <<
    	std::cos(v[3]), -std::sin(v[3]),
       	std::sin(v[3]),  std::cos(v[3]);
      Eigen::Quaterniond quat;
      quat.normalize();
      ortho_update.U = quat.toRotationMatrix();

      ortho_estimate.U = ortho_estimate.U * ortho_update.U;
      ortho_estimate.W = ortho_estimate.W * ortho_update.W;

      *this = fromOrthonormal(ortho_estimate);
      this->normalize();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Eigen::Vector4d ominus(const Line3D& line){
      /* matd_t *U_delta, *W_delta; */
      /* mline_to_orthonormal(li, &U_delta, &W_delta); */

      /* matd_t *Uj, *Wj; */
      /* mline_to_orthonormal(lj, &Uj, &Wj); */

      /* matd_get_transpose(U_delta, U_delta); */
      /* matd_get_transpose(W_delta, W_delta); */

      /* matd_product(U_delta, Uj, U_delta); */
      /* matd_product(W_delta, Wj, W_delta); */

      /* double q_delta[4]; */
      /* doubles_rot_mat_to_quat(U_delta->data, q_delta); */
      /* doubles_quat_to_rpy(q_delta, r); */
      /* r[3] = atan2(MATD_EL(W_delta, 1, 0), MATD_EL(W_delta, 0, 0)); */

      // li-1 * lj
      // transform *this and line to orthonormal
      // invert the rotation matrices associated to li (transpose)
      // get rotation deltas
      // get 3D unit q from U_delta
      // get 2D theta from W_delta
      
      OrthonormalLine3D ortho_estimate = toOrthonormal(*this);
      OrthonormalLine3D ortho_line = toOrthonormal(line);

      Eigen::Matrix2d W_delta = ortho_estimate.W.transpose() * ortho_line.W;
      Eigen::Matrix3d U_delta = ortho_estimate.U.transpose() * ortho_line.U;
      
      Eigen::Vector4d delta;
      Eigen::Quaterniond q(U_delta);
      q.normalize();
      delta[0] = q.x();
      delta[1] = q.y();
      delta[2] = q.z();
      delta[3] = atan2(W_delta(1, 0), W_delta(0, 0));
      
      return delta;
    }

  };

  G2O_TYPES_SLAM3D_ADDONS_API Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line);
  
  namespace internal {

    G2O_TYPES_SLAM3D_ADDONS_API Vector6d transformCartesianLine(const Eigen::Isometry3d& t, const Vector6d& line);
    
    G2O_TYPES_SLAM3D_ADDONS_API Vector6d normalizeCartesianLine(const Vector6d& line);

  }

}

#endif
