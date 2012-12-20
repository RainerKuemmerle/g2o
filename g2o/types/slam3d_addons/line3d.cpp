#include <Eigen/SVD>
#include "line3d.h"
namespace g2o {
  using namespace Eigen;


  Vector6d Line3D::toCartesian() const{
    Vector6d cartesian;
    cartesian.tail<3>() = d()/d().norm();
    Matrix3d A=-_skew(d());
    Eigen::JacobiSVD<Matrix3d> svd(A);
    cartesian.head<3>() = w();
    svd.solve(cartesian.head<3>());
    return cartesian;
  }

  void Line3D::jacobian(Matrix7x6d& Jp, Matrix7x6d& Jl, const Eigen::Isometry3d& x, const Line3D& l){
    Jp.setZero();
    Jl.setZero();
    Matrix6d A=Matrix6d::Zero();
    A.block<3,3>(0,0)=x.linear();
    A.block<3,3>(0,3)= _skew(x.translation())*x.linear();
    A.block<3,3>(3,3)=x.linear();
    Vector6d v=(Vector6d)l;
    Line3D lRemapped(v);

    Matrix6d D = Matrix6d::Zero();
    D.block<3,3>(0,0) = -_skew(l.d());
    D.block<3,3>(0,3) = -2*_skew(l.w());
    D.block<3,3>(3,3) = -2*_skew(l.d());
    Jp.block<6,6>(0,0) = A*D;
    
    Vector3d d = l.d();
    Vector3d w = l.w();
    double ln = d.norm();
    double iln = 1./ln;
    double iln3 = pow(iln,3);
    Matrix6d Jll;
    Jll << 
      iln,0,0,-(w.x()*d.x())*iln3,-(w.x()*d.y())*iln3,-(w.x()*d.z())*iln3,
      0,iln,0,-(w.y()*d.x())*iln3,-(w.y()*d.y())*iln3,-(w.y()*d.z())*iln3,
      0,0,iln,-(w.z()*d.x())*iln3,-(w.z()*d.y())*iln3,-(w.z()*d.z())*iln3,
      0,0,0,-(d.x()*d.x()-ln*ln)*iln3,-(d.x()*d.y())*iln3,-(d.x()*d.z())*iln3,
      0,0,0,-(d.x()*d.y())*iln3,-(d.y()*d.y()-ln*ln)*iln3,-(d.y()*d.z())*iln3,
      0,0,0,-(d.x()*d.z())*iln3,-(d.y()*d.z())*iln3,-(d.z()*d.z()-ln*ln)*iln3;
    Jl.block<6,6>(0,0) = A*Jll;
    Jl.block<1,6>(6,0) << 2*d.x(),2*d.y(),2*d.z();
  }

  Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line){
    Matrix6d A=Matrix6d::Zero();
    A.block<3,3>(0,0)=t.linear();
    A.block<3,3>(0,3)= _skew(t.translation())*t.linear();
    A.block<3,3>(3,3)=t.linear();
    Vector6d v = (Vector6d)line;
    return Line3D(A*v).normalized();
  }

}
