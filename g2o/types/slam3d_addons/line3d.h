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


  class G2O_TYPES_SLAM3D_ADDONS_API Line3D : public Vector6d{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      friend Line3D operator*(const Isometry3D& t, const Line3D& line);
      Line3D(){
        *this << 0., 0., 0., 1., 0., 0.;
      }

      Line3D(const Vector6d& v){
        (Vector6d&)*this = v;
      }

      Vector6d toCartesian() const;

      inline Vector3D w() const {
	return head<3>();
      }

      inline Vector3D d() const {
	return tail<3>();
      }

      inline void setW(const Vector3D& w_)  {
	head<3>()=w_;
      }

      inline void setD(const Vector3D& d_)  {
	tail<3>()=d_;
      }

      static inline Line3D fromCartesian(const Vector6d& cart) {
	Line3D l;
	Vector3D _d= cart.tail<3>() * 1./cart.tail<3>().norm();
	Vector3D _p= cart.head<3>();
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

      static void jacobian(Matrix7x6d& Jp, Matrix7x6d& Jl, const Isometry3D& x, const Line3D& l);
  };



  Line3D operator*(const Isometry3D& t, const Line3D& line);

  namespace internal {
    Vector6d transformCartesianLine(const Isometry3D& t, const Vector6d& line);
    Vector6d normalizeCartesianLine(const Vector6d& line);
  }

}

#endif
