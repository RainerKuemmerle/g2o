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

#include "g2o_types_slam3d_addons_api.h"
#include "g2o/stuff/misc.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {

  typedef Eigen::Matrix<double, 7, 6, Eigen::ColMajor> Matrix7x6d;

  typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;

  typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Matrix6d;

  /*Using G2O_TYPES_SLAM3D_ADDONS_API here causes Compiler Error C2487 on MSVC*/
  class Line3D : public Vector6d{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      G2O_TYPES_SLAM3D_ADDONS_API friend Line3D operator*(const Isometry3D& t, const Line3D& line);
      Line3D(){
        *this << 0., 0., 0., 1., 0., 0.;
      }

      G2O_TYPES_SLAM3D_ADDONS_API Line3D(const Vector6d& v){
        (Vector6d&)*this = v;
      }

      G2O_TYPES_SLAM3D_ADDONS_API Vector6d toCartesian() const;

      G2O_TYPES_SLAM3D_ADDONS_API inline Vector3D w() const {
	return head<3>();
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline Vector3D d() const {
	return tail<3>();
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline void setW(const Vector3D& w_)  {
	head<3>()=w_;
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline void setD(const Vector3D& d_)  {
	tail<3>()=d_;
      }

      G2O_TYPES_SLAM3D_ADDONS_API static inline Line3D fromCartesian(const Vector6d& cart) {
	Line3D l;
	Vector3D _d= cart.tail<3>() * 1./cart.tail<3>().norm();
	Vector3D _p= cart.head<3>();
	_p -= _d*(_d.dot(_p));
	l.setW(_p.cross(_p+_d));
	l.setD(_d);
	return l;
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline void normalize() {
	double n = 1./d().norm();
	(*this)*=n;
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline Line3D normalized() const {
	return  Line3D((Vector6d)(*this)*(1./d().norm()));
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline void oplus(const Vector6d& v){
	*this+=v;
	normalize();
      }

      G2O_TYPES_SLAM3D_ADDONS_API inline Vector6d ominus(const Line3D& line){
	return (Vector6d)(*this)-line;
      }

      G2O_TYPES_SLAM3D_ADDONS_API static void jacobian(Matrix7x6d& Jp, Matrix7x6d& Jl, const Isometry3D& x, const Line3D& l);
  };



  G2O_TYPES_SLAM3D_ADDONS_API Line3D operator*(const Isometry3D& t, const Line3D& line);

  namespace internal {
    G2O_TYPES_SLAM3D_ADDONS_API Vector6d transformCartesianLine(const Isometry3D& t, const Vector6d& line);
    G2O_TYPES_SLAM3D_ADDONS_API Vector6d normalizeCartesianLine(const Vector6d& line);
  }

}

#endif
