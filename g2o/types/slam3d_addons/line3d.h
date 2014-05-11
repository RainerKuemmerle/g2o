#ifndef G2O_LINE3D_H_
#define G2O_LINE3D_H_

#include "g2o/stuff/misc.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace Slam3dAddons {
  using namespace g2o;

  typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;

  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  typedef Eigen::Matrix<double, 6, 6> Matrix6d;


  class Line3D : public Vector6d{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      friend Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line);
      Line3D(){
        *this << 0., 0., 0., 1., 0., 0.;
      }

      Line3D(const Vector6d& v){
        (Vector6d&)*this = v;
      }

      Vector6d toCartesian() const;

      inline Eigen::Vector3d w() const {
	return head<3>();
      }

      inline Eigen::Vector3d d() const {
	return tail<3>();
      }

      inline void setW(const Eigen::Vector3d& w_)  {
	head<3>()=w_;
      }

      inline void setD(const Eigen::Vector3d& d_)  {
	tail<3>()=d_;
      }

      static inline Line3D fromCartesian(const Vector6d& cart) {
	Line3D l;
	Eigen::Vector3d _d= cart.tail<3>() * 1./cart.tail<3>().norm();
	Eigen::Vector3d _p= cart.head<3>();
	_p -= _d*(_d.dot(_p));
	l.setW(_p.cross(_p+_d));
	l.setD(_d);
	return l;
      }

      inline void normalize() {
	double n = 1./d().norm();
	(*this)*=n;
      }

      inline Line3D normalized() const {
	return  Line3D((Vector6d)(*this)*(1./d().norm()));
      }

      inline void oplus(Vector6d v){
	*this+=v;
	normalize();
      }

      inline Vector6d ominus(const Line3D& line){
	return (Vector6d)(*this)-line;
      }

      static void jacobian(Matrix7x6d& Jp, Matrix7x6d& Jl, const Eigen::Isometry3d& x, const Line3D& l);
  };



  Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line);

  Vector6d transformCartesianLine(const Eigen::Isometry3d& t, const Vector6d& line);
  Vector6d normalizeCartesianLine(const Vector6d& line);

}

#endif
