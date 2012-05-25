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
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"

using namespace std;
using namespace g2o;

int main (/*int argc, char** argv*/)
{
  int nodesPerLevel = 50;
  int numFloors = 50;
  double maxRadius = 100.;
  double nx = 0.01;
  double ny = 0.01;
  double nz = 0.01;
  double nqx = 0.025;
  double nqy = 0.025;
  double nqz = 0.025;

  Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
  transNoise(0,0) = pow(nx, 2);
  transNoise(1,1) = pow(ny, 2);
  transNoise(2,2) = pow(nz, 2);

  Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
  rotNoise(0,0) = pow(nqx, 2);
  rotNoise(1,1) = pow(nqy, 2);
  rotNoise(2,2) = pow(nqz, 2);

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.block<3,3>(0,0) = transNoise;
  information.block<3,3>(3,3) = rotNoise;

  vector<VertexSE3*> vertices;
  vector<EdgeSE3*> odometryEdges;	
  vector<EdgeSE3*> edges;	
  int id = 0;
  for (int f = 0; f < numFloors; ++f){
    for (int n = 0; n < nodesPerLevel; ++n) {
      VertexSE3* v = new VertexSE3;
      v->setId(id++);

      Eigen::AngleAxisd rotz(-M_PI + 2*n*M_PI / nodesPerLevel, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd roty(-0.5*M_PI + id*M_PI / (numFloors * nodesPerLevel), Eigen::Vector3d::UnitY());
      Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();

      Eigen::Isometry3d t;
      t = rot;
      t.translation() = t.linear() * Eigen::Vector3d(maxRadius, 0, 0);
      v->setEstimate(t);
      vertices.push_back(v);
    }
  }

  // generate odometry edges
  for (size_t i = 1; i < vertices.size(); ++i) {
    VertexSE3* prev = vertices[i-1];
    VertexSE3* cur  = vertices[i];
    Eigen::Isometry3d t = prev->estimate().inverse() * cur->estimate();
    EdgeSE3* e = new EdgeSE3;
    e->setVertex(0, prev);
    e->setVertex(1, cur);
    e->setMeasurement(t);
    e->setInformation(information);
    odometryEdges.push_back(e);
    edges.push_back(e);
  }

  // generate loop closure edges
  for (int f = 1; f < numFloors; ++f) {
    for (int nn = 0; nn < nodesPerLevel; ++nn) {
      VertexSE3* from = vertices[(f-1)*nodesPerLevel + nn];
      for (int n = -1; n <= 1; ++n) {
        if (f == numFloors-1 && n == 1)
          continue;
        VertexSE3* to   = vertices[f*nodesPerLevel + nn + n];
        Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
        EdgeSE3* e = new EdgeSE3;
        e->setVertex(0, from);
        e->setVertex(1, to);
        e->setMeasurement(t);
        e->setInformation(information);
        edges.push_back(e);
      }
    }
  }

  GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> transSampler;
  transSampler.setDistribution(transNoise);
  GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> rotSampler;
  rotSampler.setDistribution(rotNoise);

  // noise for all the edges
  for (size_t i = 0; i < edges.size(); ++i) {
    EdgeSE3* e = edges[i];
    Eigen::Quaterniond gtQuat = (Eigen::Quaterniond)e->measurement().linear();
    Eigen::Vector3d gtTrans = e->measurement().translation();

    Eigen::Vector3d quatXYZ = rotSampler.generateSample();
    double qw = 1.0 - quatXYZ.norm();
    if (qw < 0) {
      qw = 0.;
      cerr << "x";
    }
    Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());
    rot.normalize();
    Eigen::Vector3d trans = transSampler.generateSample();
    rot = gtQuat * rot;
    trans = gtTrans + trans;

    Eigen::Isometry3d noisyMeasurement = (Eigen::Isometry3d) rot;
    noisyMeasurement.translation() = trans;
    e->setMeasurement(noisyMeasurement);
  }

  // concatenate all the odometry constraints
  for (size_t i =0; i < odometryEdges.size(); ++i) {
    EdgeSE3* e = edges[i];
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
    HyperGraph::VertexSet aux; aux.insert(from);
    e->initialEstimate(aux, to);
  }

  // write output
  ofstream fout("sphere.g2o");
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexSE3* v = vertices[i];
    fout << "VERTEX_SE3:QUAT " << v->id() << " ";
    v->write(fout);
    fout << endl;
  }

  for (size_t i = 0; i < edges.size(); ++i) {
    EdgeSE3* e = edges[i];
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
    fout << "EDGE_SE3:QUAT " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << endl;
  }

  return 0;
}
