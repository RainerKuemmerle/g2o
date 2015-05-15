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

#include "line3d.h"
#include <iostream>

namespace g2o {
  using namespace std;
  using namespace Eigen;

  static inline void _skew(Matrix3D& S, const Vector3D& t){
    S <<
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y()     ,t.x(),   0;
  }

  static inline Matrix3D _skew(const Vector3D& t){
    Matrix3D S;
    S <<
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y(),     t.x(),   0;
    return S;
  }


  Vector6d Line3D::toCartesian() const{
    Vector6d cartesian;
    cartesian.tail<3>() = d()/d().norm();
    Matrix3D W=-_skew(d());
    double damping = 1e-9;
    Matrix3D A = W.transpose()*W+(Matrix3D::Identity()*damping);
    cartesian.head<3>() = A.ldlt().solve(W.transpose()*w());
    return cartesian;
  }

  void Line3D::jacobian(Matrix7x6d& Jp, Matrix7x6d& Jl, const Isometry3D& x, const Line3D& l){
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

    Vector3D d = l.d();
    Vector3D w = l.w();
    double ln = d.norm();
    double iln = 1./ln;
    double iln3 = std::pow(iln,3);
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

  Line3D operator*(const Isometry3D& t, const Line3D& line){
    Matrix6d A=Matrix6d::Zero();
    A.block<3,3>(0,0)=t.linear();
    A.block<3,3>(0,3)= _skew(t.translation())*t.linear();
    A.block<3,3>(3,3)=t.linear();
    Vector6d v = (Vector6d)line;
    // cout << "v" << endl << v << endl;
    // cout << "A" << endl << A << endl;
    // cout << "Av" << endl << A*v << endl;
    return Line3D(A*v);
  }

  namespace internal
  {
    Vector6d transformCartesianLine(const Isometry3D& t, const Vector6d& line){
      Vector6d l;
      l.head<3>() = t*line.head<3>();
      l.tail<3>() = t.linear()*line.tail<3>();
      return normalizeCartesianLine(l);
    }

    Vector6d normalizeCartesianLine(const Vector6d& line) {
      Vector3D p0 = line.head<3>();
      Vector3D d0 = line.tail<3>();
      d0.normalize();
      p0-=d0*(d0.dot(p0));
      Vector6d nl;
      nl.head<3>()=p0;
      nl.tail<3>()=d0;
      return nl;
    }
  } // end namespace internal

} // end namespace g2o
