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

int main(int , char** ){

Matrix3d I;
  Matrix3d R = Matrix3d::Identity();
  Matrix3d rot = (Matrix3d)AngleAxisd(0.01, Vector3d::UnitZ());
  rot = rot * (Matrix3d)AngleAxisd(0.01, Vector3d::UnitX());
  
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
  Eigen::Matrix3d approxSolution = R;
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

 }
