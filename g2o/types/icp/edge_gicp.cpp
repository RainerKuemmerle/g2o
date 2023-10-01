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

#include "edge_gicp.h"

#include <Eigen/Geometry>

namespace g2o {

EdgeGICP::EdgeGICP() {
  pos0.setZero();
  pos1.setZero();
  normal0 << 0, 0, 1;
  normal1 << 0, 0, 1;
  // makeRot();
  R0.setIdentity();
  R1.setIdentity();
}

void EdgeGICP::makeRot0() {
  Vector3 y;
  y << 0, 1, 0;
  R0.row(2) = normal0;
  y = y - normal0(1) * normal0;
  y.normalize();  // need to check if y is close to 0
  R0.row(1) = y;
  R0.row(0) = normal0.cross(R0.row(1));
  //      cout << normal.transpose() << endl;
  //      cout << R0 << endl << endl;
  //      cout << R0*R0.transpose() << endl << endl;
}

void EdgeGICP::makeRot1() {
  Vector3 y;
  y << 0, 1, 0;
  R1.row(2) = normal1;
  y = y - normal1(1) * normal1;
  y.normalize();  // need to check if y is close to 0
  R1.row(1) = y;
  R1.row(0) = normal1.cross(R1.row(1));
}

Matrix3 EdgeGICP::prec0(double e) {
  makeRot0();
  Matrix3 prec;
  prec << e, 0, 0, 0, e, 0, 0, 0, 1;
  return R0.transpose() * prec * R0;
}

Matrix3 EdgeGICP::prec1(double e) {
  makeRot1();
  Matrix3 prec;
  prec << e, 0, 0, 0, e, 0, 0, 0, 1;
  return R1.transpose() * prec * R1;
}

Matrix3 EdgeGICP::cov0(double e) {
  makeRot0();
  Matrix3 cov;
  cov << 1, 0, 0, 0, 1, 0, 0, 0, e;
  return R0.transpose() * cov * R0;
}

Matrix3 EdgeGICP::cov1(double e) {
  makeRot1();
  Matrix3 cov;
  cov << 1, 0, 0, 0, 1, 0, 0, 0, e;
  return R1.transpose() * cov * R1;
}

}  // namespace g2o
