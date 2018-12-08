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


#include <Eigen/Core>

#include "g2o_types_slam3d_addons_api.h"

namespace g2o {
  
  typedef Eigen::Matrix<number_t, 6, 1> Vector6;
  typedef Eigen::Matrix<number_t, 6, 6> Matrix6;
  typedef Eigen::Matrix<number_t, 6, 4> Matrix6x4;
  
  struct OrthonormalLine3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Matrix2 W;
    Matrix3 U;

    OrthonormalLine3D() {
      W = Matrix2::Identity();
      U = Matrix3::Identity();
    }
  };
  typedef struct OrthonormalLine3D OrthonormalLine3D;
  
  class Line3D : public Vector6 {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    G2O_TYPES_SLAM3D_ADDONS_API friend Line3D operator*(const Isometry3& t,
    							const Line3D& line);

    G2O_TYPES_SLAM3D_ADDONS_API Line3D() {
      *this << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    }

    G2O_TYPES_SLAM3D_ADDONS_API Line3D(const Vector6& v) {
      (Vector6&)*this = v;
    }

    G2O_TYPES_SLAM3D_ADDONS_API Vector6 toCartesian() const;

    G2O_TYPES_SLAM3D_ADDONS_API inline Vector3 w() const {
      return head<3>();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Vector3 d() const {
      return tail<3>();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void setW(const Vector3& w_) {
      head<3>() = w_;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void setD(const Vector3& d_) {
      tail<3>() = d_;
    }

    G2O_TYPES_SLAM3D_ADDONS_API static inline Line3D fromCartesian(const Vector6& cart) {
      Line3D l;
      Vector3 _p = cart.head<3>();
      Vector3 _d = cart.tail<3>() * 1.0/cart.tail<3>().norm();
      _p -= _d*(_d.dot(_p));
      l.setW(_p.cross(_p+_d));
      l.setD(_d);
      return l;
    }

    G2O_TYPES_SLAM3D_ADDONS_API static inline Line3D fromOrthonormal(const OrthonormalLine3D& ortho) {
      Vector3 w;
      w.x() = ortho.U(0, 0) * ortho.W(0, 0);
      w.y() = ortho.U(1, 0) * ortho.W(0, 0);
      w.z() = ortho.U(2, 0) * ortho.W(0, 0);

      Vector3 d;
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
      OrthonormalLine3D ortho;

      Vector2 mags;
      mags << line.d().norm(), line.w().norm();

      number_t wn = 1.0 / mags.norm();
      ortho.W <<
      	mags.y() * wn, -mags.x() * wn,
      	mags.x() * wn,  mags.y() * wn;

      number_t mn = 1.0 / mags.y();
      number_t dn = 1.0 / mags.x();
      Vector3 mdcross;
      mdcross = line.w().cross(line.d());
      number_t mdcrossn = 1.0 / mdcross.norm();
      ortho.U <<
      	line.w().x() * mn, line.d().x() * dn, mdcross.x() * mdcrossn,
      	line.w().y() * mn, line.d().y() * dn, mdcross.y() * mdcrossn,
      	line.w().z() * mn, line.d().z() * dn, mdcross.z() * mdcrossn;

      return ortho;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void normalize() {
      number_t n = 1.0/d().norm();
      (*this)*=n;
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Line3D normalized() const {
      return Line3D((Vector6)(*this)*(1.0/d().norm()));
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline void oplus(const Vector4& v){
      OrthonormalLine3D ortho_estimate = toOrthonormal(*this);
      OrthonormalLine3D ortho_update;
      ortho_update.W <<
    	std::cos(v[3]), -std::sin(v[3]),
       	std::sin(v[3]),  std::cos(v[3]);
      Quaternion quat(std::sqrt(1 - v.head<3>().squaredNorm()), v[0], v[1], v[2]);
      quat.normalize();
      ortho_update.U = quat.toRotationMatrix();

      ortho_estimate.U = ortho_estimate.U * ortho_update.U;
      ortho_estimate.W = ortho_estimate.W * ortho_update.W;

      *this = fromOrthonormal(ortho_estimate);
      this->normalize();
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline Vector4 ominus(const Line3D& line) {
      OrthonormalLine3D ortho_estimate = toOrthonormal(*this);
      OrthonormalLine3D ortho_line = toOrthonormal(line);

      Matrix2 W_delta = ortho_estimate.W.transpose() * ortho_line.W;
      Matrix3 U_delta = ortho_estimate.U.transpose() * ortho_line.U;
	
      Vector4 delta;
      Quaternion q(U_delta);
      q.normalize();
      delta[0] = q.x();
      delta[1] = q.y();
      delta[2] = q.z();
      delta[3] = std::atan2(W_delta(1, 0), W_delta(0, 0));
     
      return delta;
    }

  };

  G2O_TYPES_SLAM3D_ADDONS_API Line3D operator*(const Isometry3& t, const Line3D& line);
  
  namespace internal {

    G2O_TYPES_SLAM3D_ADDONS_API Vector6 transformCartesianLine(const Isometry3& t, const Vector6& line);
    
    G2O_TYPES_SLAM3D_ADDONS_API Vector6 normalizeCartesianLine(const Vector6& line);

    static inline number_t mline_elevation(const number_t v[3]) {
      return std::atan2(v[2], sqrt(v[0]*v[0] + v[1]*v[1]));
    }
    
    G2O_TYPES_SLAM3D_ADDONS_API inline number_t getAzimuth(const Vector3& direction) {
      return std::atan2(direction.y(), direction.x());
    }

    G2O_TYPES_SLAM3D_ADDONS_API inline number_t getElevation(const Vector3& direction) {
      return std::atan2(direction.z(), direction.head<2>().norm());
    }
    
  }

}

#endif
