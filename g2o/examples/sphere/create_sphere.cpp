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

#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o {

static int create_sphere(int argc, char** argv) {
  // command line parsing
  int nodesPerLevel;
  int numLaps;
  bool randomSeed;
  double radius;
  std::vector<double> noiseTranslation;
  std::vector<double> noiseRotation;
  std::string outFilename;
  CommandArgs arg;
  arg.param("o", outFilename, "-", "output filename");
  arg.param("nodesPerLevel", nodesPerLevel, 50,
            "how many nodes per lap on the sphere");
  arg.param("laps", numLaps, 50,
            "how many times the robot travels around the sphere");
  arg.param("radius", radius, 100., "radius of the sphere");
  arg.param("noiseTranslation", noiseTranslation, std::vector<double>(),
            "set the noise level for the translation, separated by semicolons "
            "without spaces e.g: "
            "\"0.1;0.1;0.1\"");
  arg.param("noiseRotation", noiseRotation, std::vector<double>(),
            "set the noise level for the rotation, separated by semicolons "
            "without spaces e.g: "
            "\"0.001;0.001;0.001\"");
  arg.param("randomSeed", randomSeed, false,
            "use a randomized seed for generating the sphere");
  arg.parseArgs(argc, argv);

  if (noiseTranslation.empty()) {
    std::cerr << "using default noise for the translation" << std::endl;
    noiseTranslation.push_back(0.01);
    noiseTranslation.push_back(0.01);
    noiseTranslation.push_back(0.01);
  }
  std::cerr << "Noise for the translation:";
  for (double i : noiseTranslation) std::cerr << " " << i;
  std::cerr << std::endl;
  if (noiseRotation.empty()) {
    std::cerr << "using default noise for the rotation" << std::endl;
    noiseRotation.push_back(0.005);
    noiseRotation.push_back(0.005);
    noiseRotation.push_back(0.005);
  }
  std::cerr << "Noise for the rotation:";
  for (double i : noiseRotation) std::cerr << " " << i;
  std::cerr << std::endl;

  Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i)
    transNoise(i, i) = std::pow(noiseTranslation[i], 2);

  Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i) rotNoise(i, i) = std::pow(noiseRotation[i], 2);

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.block<3, 3>(0, 0) = transNoise.inverse();
  information.block<3, 3>(3, 3) = rotNoise.inverse();

  std::vector<std::shared_ptr<VertexSE3>> vertices;
  std::vector<std::shared_ptr<EdgeSE3>> odometryEdges;
  std::vector<std::shared_ptr<EdgeSE3>> edges;
  int id = 0;
  for (int f = 0; f < numLaps; ++f) {
    for (int n = 0; n < nodesPerLevel; ++n) {
      auto v = std::make_shared<VertexSE3>();
      v->setId(id++);

      Eigen::AngleAxisd rotz(-M_PI + 2 * n * M_PI / nodesPerLevel,
                             Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd roty(
          -0.5 * M_PI + id * M_PI / (numLaps * nodesPerLevel),
          Eigen::Vector3d::UnitY());
      Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();

      Eigen::Isometry3d t;
      t = rot;
      t.translation() = t.linear() * Eigen::Vector3d(radius, 0, 0);
      v->setEstimate(t);
      vertices.emplace_back(v);
    }
  }

  // generate odometry edges
  for (size_t i = 1; i < vertices.size(); ++i) {
    const auto& prev = vertices[i - 1];
    const auto& cur = vertices[i];
    Eigen::Isometry3d t = prev->estimate().inverse() * cur->estimate();
    auto e = std::make_shared<EdgeSE3>();
    e->setVertex(0, prev);
    e->setVertex(1, cur);
    e->setMeasurement(t);
    e->setInformation(information);
    odometryEdges.push_back(e);
    edges.push_back(e);
  }

  // generate loop closure edges
  for (int f = 1; f < numLaps; ++f) {
    for (int nn = 0; nn < nodesPerLevel; ++nn) {
      auto from = vertices[(f - 1) * nodesPerLevel + nn];
      for (int n = -1; n <= 1; ++n) {
        if (f == numLaps - 1 && n == 1) continue;
        auto to = vertices[f * nodesPerLevel + nn + n];
        Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
        auto e = std::make_shared<EdgeSE3>();
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

  if (randomSeed) {
    std::random_device r;
    std::seed_seq seedSeq{r(), r(), r(), r(), r()};
    std::vector<int> seeds(2);
    seedSeq.generate(seeds.begin(), seeds.end());
    std::cerr << "using seeds:";
    for (int seed : seeds) std::cerr << " " << seed;
    std::cerr << std::endl;
    transSampler.seed(seeds[0]);
    rotSampler.seed(seeds[1]);
  }

  // noise for all the edges
  for (auto& e : edges) {
    Eigen::Quaterniond gtQuat = Eigen::Quaterniond(e->measurement().linear());
    Eigen::Vector3d gtTrans = e->measurement().translation();

    Eigen::Vector3d quatXYZ = rotSampler.generateSample();
    double qw = 1.0 - quatXYZ.norm();
    if (qw < 0) {
      qw = 0.;
      std::cerr << "x";
    }
    Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());
    rot.normalize();
    Eigen::Vector3d trans = transSampler.generateSample();
    rot = gtQuat * rot;
    trans = gtTrans + trans;

    Eigen::Isometry3d noisyMeasurement = Eigen::Isometry3d(rot);
    noisyMeasurement.translation() = trans;
    e->setMeasurement(noisyMeasurement);
  }

  // concatenate all the odometry constraints to compute the initial state
  for (auto& e : odometryEdges) {
    auto from = std::static_pointer_cast<VertexSE3>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1).get());
    HyperGraph::VertexSet aux;
    aux.insert(from);
    e->initialEstimate(aux, to);
  }

  // write output
  std::ofstream fileOutputStream;
  if (outFilename != "-") {
    std::cerr << "Writing into " << outFilename << std::endl;
    fileOutputStream.open(outFilename.c_str());
  } else {
    std::cerr << "writing to stdout" << std::endl;
  }

  std::string vertexTag = Factory::instance()->tag(vertices[0].get());
  std::string edgeTag = Factory::instance()->tag(edges[0].get());

  std::ostream& fout = outFilename != "-" ? fileOutputStream : std::cout;
  for (auto& vertice : vertices) {
    VertexSE3* v = vertice.get();
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << std::endl;
  }

  for (auto& edge : edges) {
    EdgeSE3* e = edge.get();
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0).get());
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1).get());
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << std::endl;
  }

  return 0;
}
}  // namespace g2o

int main(int argc, char** argv) { return g2o::create_sphere(argc, argv); }
