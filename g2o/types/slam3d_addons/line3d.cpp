#include <Eigen/Dense>
#include "line3d.h"
#include <iostream>

namespace Slam3dAddons {
  using namespace g2o;
  using namespace Eigen;
  using namespace std;

  inline void _skew(Eigen::Matrix3d& S, const Eigen::Vector3d& t){
    S <<   
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y()     ,t.x(),   0;
  }

  inline Eigen::Matrix3d _skew(const Eigen::Vector3d& t){
    Eigen::Matrix3d S;
    S <<   
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y(),     t.x(),   0;
    return S;
  }


  Vector6d Line3D::toCartesian() const{
    Vector6d cartesian;
    cartesian.tail<3>() = d()/d().norm();
    Matrix3d W=-_skew(d());
    double damping = 1e-9;
    Matrix3d A = W.transpose()*W+(Matrix3d::Identity()*damping);
    cartesian.head<3>() = A.ldlt().solve(W.transpose()*w());
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
    cout << "v" << endl << v << endl;
    cout << "A" << endl << A << endl;
    cout << "Av" << endl << A*v << endl;
    return Line3D(A*v);
  }

  Vector6d transformCartesianLine(const Eigen::Isometry3d& t, const Vector6d& line){
    Vector6d l;
    l.head<3>() = t*line.head<3>();
    l.tail<3>() = t.linear()*line.tail<3>();
    return normalizeCartesianLine(l);
  }

  Vector6d normalizeCartesianLine(const Vector6d& line) {
    Eigen::Vector3d p0 = line.head<3>();
    Eigen::Vector3d d0 = line.tail<3>();
    d0.normalize();
    p0-=d0*(d0.dot(p0));
    Vector6d nl;
    nl.head<3>()=p0;
    nl.tail<3>()=d0;
    return nl;
  }

}
