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

#include <iostream>

#include "line3d.h"

namespace g2o {

  using namespace std;
  using namespace Eigen;

  static inline void _skew(Matrix3d& S, const Vector3d& t) {
    S <<
           0, -t.z(),  t.y(),
       t.z(),      0, -t.x(),
      -t.y(),  t.x(),      0;
  }

  static inline Matrix3d _skew(const Vector3d& t) {
    Matrix3d S;
    S <<
           0, -t.z(),  t.y(),
       t.z(),      0, -t.x(),
      -t.y(),  t.x(),      0;
    return S;
  }


  Vector6d Line3D::toCartesian() const {
    Vector6d cartesian;
    cartesian.tail<3>() = d()/d().norm();
    Eigen::Matrix3d W = -_skew(d());
    double damping = 1e-9;
    Eigen::Matrix3d A = W.transpose()*W + (Eigen::Matrix3d::Identity()*damping);
    cartesian.head<3>() = A.ldlt().solve(W.transpose()*w());
    return cartesian;
  }

  Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line){
    Matrix6d A = Matrix6d::Zero();
    A.block<3, 3>(0, 0) = t.linear();
    A.block<3, 3>(0, 3) = _skew(t.translation())*t.linear();
    A.block<3, 3>(3, 3) = t.linear();
    Vector6d v = (Vector6d)line;
    return Line3D(A*v);
  }
  
  namespace internal {
    Vector6d transformCartesianLine(const Eigen::Isometry3d& t, const Vector6d& line) {
      Vector6d l;
      l.head<3>() = t*line.head<3>();
      l.tail<3>() = t.linear()*line.tail<3>();
      return normalizeCartesianLine(l);
    }

    Vector6d normalizeCartesianLine(const Vector6d& line) {
      Vector3d p0 = line.head<3>();
      Vector3d d0 = line.tail<3>();
      d0.normalize();
      p0 -= d0*(d0.dot(p0));
      Vector6d nl;
      nl.head<3>() = p0;
      nl.tail<3>() = d0;
      return nl;
    }
    
  }

}
