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
#include "isometry3d_mappings.h"
#include "g2o/stuff/macros.h"

#include <cstdio>

using namespace std;
using namespace g2o;
using namespace g2o::internal;
using namespace Eigen;

int main(int , char** ){

Matrix3D I;
  Matrix3D R = Matrix3D::Identity();
  Matrix3D rot = (Matrix3D)AngleAxisd(0.01, Vector3D::UnitZ());
  rot = rot * (Matrix3D)AngleAxisd(0.01, Vector3D::UnitX());
  
  cerr << "Initial rotation matrix accuracy" << endl;
  I = R * R.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      printf("%.30f   ", I(i,j));
    }
    printf("\n");
  }

  cerr << "After further multiplications" << endl;
  for (int i = 0; i < 10000; ++i)
    R = R * rot;
  I = R * R.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      printf("%.30f   ", I(i,j));
    }
    printf("\n");
  }

  cerr << PVAR(R) << endl;
  printf("det %.30f\n", R.determinant());
  printf("\nUsing nearest orthogonal matrix\n");
  Matrix3D approxSolution = R;
  approximateNearestOrthogonalMatrix(approxSolution);
  nearestOrthogonalMatrix(R);
  cerr << PVAR(R) << endl;
  printf("det %.30f\n", R.determinant());
  I = R * R.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      printf("%.30f   ", I(i,j));
    }
    printf("\n");
  }
  cerr << "Norm of the columns" << endl;
  for (int i = 0; i < 3; ++i)
    printf("%.30f   ", R.col(i).norm());
  printf("\nUsing approximate nearest orthogonal matrix\n");
  I = approxSolution * approxSolution.transpose();
  cerr << PVAR(approxSolution) << endl;
  printf("det %.30f\n", approxSolution.determinant());
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      printf("%.30f   ", I(i,j));
    }
    printf("\n");
  }
  cerr << "Norm of the columns" << endl;
  for (int i = 0; i < 3; ++i)
    printf("%.30f   ", approxSolution.col(i).norm());

  cerr << endl;
  return 0;

  Vector3D eulerAngles(.1,.2,.3);
  Matrix3D m1=fromEuler(eulerAngles);
  cerr << "m1=fromEuler(eulerAngles)" << endl;
  cerr << "eulerAngles:" << endl;
  cerr << eulerAngles << endl;
  cerr << "m1:" << endl;
  cerr << m1 << endl;

  Vector3D eulerAngles1 =  toEuler(m1);
  cerr << "eulerAngles1 =  toEuler(m1) " << endl;
  cerr << "eulerAngles1:" << endl;
  cerr << eulerAngles1 << endl;

  Vector3D q=toCompactQuaternion(m1);
  cerr << "q=toCompactQuaternion(m1)" << endl;
  cerr << "q:" << endl;
  cerr <<  q << endl;

  Matrix3D m2=fromCompactQuaternion(q);
  cerr << "m2=fromCompactQuaternion(q);" << endl;
  cerr << "m2:" << endl;
  cerr << m2 << endl;

  Vector6d et;
  Vector3D t(1.,2.,3.);
  et.block<3,1>(0,0)=eulerAngles;
  et.block<3,1>(3,0)=t;
  Isometry3D i1 = fromVectorET(et);
  cerr << "i1 = fromVectorET(et);" << endl;
  cerr << "et:" << endl;
  cerr << et << endl;
  cerr << "i1" << endl;
  cerr << i1.rotation() << endl;
  cerr << i1.translation() << endl;
  Vector6d et2=toVectorET(i1);
  cerr << "et2=toVectorET(i1);" << endl;
  cerr << "et2" << endl;
  cerr << et2 << endl;

  Vector6d qt1=toVectorMQT(i1);
  cerr << "qt1=toVectorMQT(i1)" << endl;
  cerr << "qt1:" << endl;
  cerr << qt1 << endl;

  Isometry3D i2 = fromVectorMQT(qt1);
  cerr << "i2 = fromVectorMQT(qt1)" << endl;
  cerr << "i2" << endl;
  cerr << i2.rotation() << endl;
  cerr << i2.translation() << endl;

  Vector7d qt2=toVectorQT(i1);
  cerr << "qt2=toVectorQT(i1)" << endl;
  cerr << "qt2:" << endl;
  cerr << qt2 << endl;

  Isometry3D i3 = fromVectorQT(qt2);
  cerr << "i3 = fromVectorQT(qt2)" << endl;
  cerr << "i3" << endl;
  cerr << i3.rotation() << endl;
  cerr << i3.translation() << endl;

}
