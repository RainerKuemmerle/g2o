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
#include <cstdio>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/stuff/macros.h"
#include "edge_se3.h"
#include "edge_se3_prior.h"

using namespace std;
using namespace g2o;

Eigen::Isometry3d randomIsometry3d()
{
  Eigen::Vector3d rotAxisAngle = Vector3d::Random();
  rotAxisAngle += Vector3d::Random();
  Eigen::AngleAxisd rotation(rotAxisAngle.norm(), rotAxisAngle.normalized());
  Eigen::Isometry3d result = (Eigen::Isometry3d)rotation.toRotationMatrix();
  result.translation() = Vector3d::Random();
  return result;
}

int main(int , char** )
{
  if (0) {
    OptimizableGraph graph;
    ParameterSE3Offset* offsetParam = new ParameterSE3Offset;
    offsetParam->setId(0);
    graph.addParameter(offsetParam);

    Eigen::Isometry3d zeroPose = Eigen::Isometry3d::Identity();
    VertexSE3* v1 = new VertexSE3;
    v1->setId(0);
    graph.addVertex(v1);
    OptimizableGraph::VertexSet auxSet;
    EdgeSE3Prior* priorEdge = new EdgeSE3Prior;
    priorEdge->setVertex(0, v1);
    priorEdge->setMeasurement(randomIsometry3d());
    priorEdge->setParameterId(0, 0);
    cout << PVAR(priorEdge->measurement().matrix()) << endl;
    graph.addEdge(priorEdge);

    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Identity();

    cout << "Full information" << endl;
    v1->setEstimate(zeroPose);
    priorEdge->setInformation(information);
    priorEdge->initialEstimate(auxSet, 0);
    cout << PVAR(priorEdge->chi2()) << endl;
    cout << v1->estimate().matrix() << endl << endl;
    priorEdge->computeError();

    cout << "Only translation" << endl;
    v1->setEstimate(zeroPose);
    information.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    information.block<3,3>(3,3) = Eigen::Matrix3d::Zero();
    priorEdge->setInformation(information);
    priorEdge->initialEstimate(auxSet, 0);
    cout << PVAR(priorEdge->chi2()) << endl;
    cout << v1->estimate().matrix() << endl << endl;
    priorEdge->computeError();

    cout << "Only rotation" << endl;
    v1->setEstimate(zeroPose);
    information.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
    information.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    priorEdge->setInformation(information);
    priorEdge->initialEstimate(auxSet, 0);
    cout << PVAR(priorEdge->chi2()) << endl;
    cout << v1->estimate().matrix() << endl << endl;
    priorEdge->computeError();
    return 0;
  }

  VertexSE3 v1;
  v1.setId(0); 

  VertexSE3 v2;
  v2.setId(1); 

  EdgeSE3 e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(Eigen::Matrix<double,6,6>::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 100000; ++k) {

    v1.setEstimate(randomIsometry3d());
    v2.setEstimate(randomIsometry3d());
    e.setMeasurement(randomIsometry3d());

    // calling the analytic Jacobian but writing to the numeric workspace
    e.BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3>::linearizeOplus(numericJacobianWorkspace);
    // copy result into analytic workspace
    jacobianWorkspace = numericJacobianWorkspace;

    // compute the numeric Jacobian into the numericJacobianWorkspace workspace as setup by the previous call
    e.BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3>::linearizeOplus();

    // compare the two Jacobians
    const double allowedDifference = 1e-6;
    for (int i = 0; i < 2; ++i) {
      double* n = numericJacobianWorkspace.workspaceForVertex(i);
      double* a = jacobianWorkspace.workspaceForVertex(i);
      for (int j = 0; j < 6*6; ++j) {
        double d = fabs(n[j] - a[j]);
        if (d > allowedDifference) {
          cerr << "\ndetected difference in the Jacobians " << d << endl;
          cerr << PVAR(v1.estimate().matrix()) << endl << endl;
          cerr << PVAR(v2.estimate().matrix()) << endl << endl;
          cerr << PVAR(e.measurement().matrix()) << endl << endl;
          return 1;
        }
      }
    }
    cerr << "+";

  }
  return 0;
}
