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

#include "CLI/CLI.hpp"
#include "g2o/core/abstract_graph.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/core/io/io_format.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o {

namespace {
int create_sphere(int argc, char** argv) {
  // command line parsing
  int nodesPerLevel = 50;
  int numLaps = 50;
  bool randomSeed = false;
  double radius = 100.;
  std::vector<double> noiseTranslation = {0.01, 0.01, 0.01};
  std::vector<double> noiseRotation = {0.005, 0.005, 0.005};
  std::string outFilename = "-";

  CLI::App app{"g2o: Create a randomized sphere"};
  app.add_option("-o,--output", outFilename,
                 "output filename (use - for stdout)")
      ->capture_default_str();
  app.add_option("--nodes_per_Level", nodesPerLevel,
                 "how many nodes per lap on the sphere")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  app.add_option("--laps", numLaps,
                 "how many times the robot travels around the sphere")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  app.add_option("--radius", radius, "radius of the sphere")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  app.add_option("--noise_translation", noiseTranslation,
                 "set the noise level for the translation")
      ->expected(3)
      ->capture_default_str();
  app.add_option("--noise_rotation", noiseRotation,
                 "set the noise level for the rotation")
      ->capture_default_str()
      ->expected(3);
  app.add_flag("--random_seed", randomSeed,
               "use a randomized seed for generating the sphere");
  CLI11_PARSE(app, argc, argv);

  std::cerr << "Noise for the translation:";
  for (const double i : noiseTranslation) std::cerr << " " << i;
  std::cerr << '\n';
  std::cerr << "Noise for the rotation:";
  for (const double i : noiseRotation) std::cerr << " " << i;
  std::cerr << '\n';

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

      const Eigen::AngleAxisd rotz(-M_PI + (2 * n * M_PI / nodesPerLevel),
                                   Eigen::Vector3d::UnitZ());
      const Eigen::AngleAxisd roty(
          (-0.5 * M_PI) + (id * M_PI / (numLaps * nodesPerLevel)),
          Eigen::Vector3d::UnitY());
      const Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();

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
    const Eigen::Isometry3d t = prev->estimate().inverse() * cur->estimate();
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
      const auto& from = vertices[((f - 1) * nodesPerLevel) + nn];
      for (int n = -1; n <= 1; ++n) {
        if (f == numLaps - 1 && n == 1) continue;
        const auto& to = vertices[(f * nodesPerLevel) + nn + n];
        const Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
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
    std::vector<unsigned int> seeds(2);
    seedSeq.generate(seeds.begin(), seeds.end());
    std::cerr << "using seeds:";
    for (const int seed : seeds) std::cerr << " " << seed;
    std::cerr << '\n';
    transSampler.seed(seeds[0]);
    rotSampler.seed(seeds[1]);
  }

  // noise for all the edges
  for (auto& e : edges) {
    const Eigen::Quaterniond gtQuat =
        Eigen::Quaterniond(e->measurement().linear());
    const Eigen::Vector3d gtTrans = e->measurement().translation();

    const Eigen::Vector3d quatXYZ = rotSampler.generateSample();
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

    Eigen::Isometry3d noisyMeasurement(rot);
    noisyMeasurement.translation() = trans;
    e->setMeasurement(noisyMeasurement);
  }

  // concatenate all the odometry constraints to compute the initial state
  for (auto& e : odometryEdges) {
    auto from = e->vertexXn<0>();
    auto to = e->vertexXn<1>();
    HyperGraph::VertexSet aux;
    aux.insert(from);
    e->initialEstimate(aux, to.get());
  }

  const std::string vertexTag = Factory::instance()->tag(vertices[0].get());
  const std::string edgeTag = Factory::instance()->tag(edges[0].get());

  g2o::AbstractGraph output;
  for (auto& vertex : vertices) {
    std::vector<double> vertex_estimate;
    vertex->getEstimateData(vertex_estimate);
    g2o::AbstractGraph output;
    output.vertices().emplace_back(vertexTag, vertex->id(), vertex_estimate);
  }

  for (auto& edge : edges) {
    auto from = edge->vertexXn<0>();
    auto to = edge->vertexXn<1>();

    std::vector<int> ids = {from->id(), to->id()};
    std::vector<double> data(edge->measurementDimension());
    edge->getMeasurementData(data.data());
    MatrixX::MapType information(edge->informationData(), edge->dimension(),
                                 edge->dimension());
    std::vector<double> upper_triangle;
    upper_triangle.reserve((edge->dimension() * (edge->dimension() + 1)) / 2);
    for (int r = 0; r < edge->dimension(); ++r)
      for (int c = r; c < edge->dimension(); ++c)
        upper_triangle.push_back(information(r, c));
    output.edges().emplace_back(edgeTag, ids, data, upper_triangle,
                                edge->parameterIds());
  }

  // write output
  std::ofstream fileOutputStream;
  if (outFilename != "-") {
    std::cerr << "Writing into " << outFilename << '\n';
    fileOutputStream.open(outFilename.c_str());
  } else {
    std::cerr << "writing to stdout\n";
  }

  std::ostream& fout = outFilename != "-" ? fileOutputStream : std::cout;
  if (!output.save(fout, g2o::io::Format::kG2O)) {
    std::cerr << "Error while writing\n";
  }

  return 0;
}
}  // namespace
}  // namespace g2o

int main(int argc, char** argv) { return g2o::create_sphere(argc, argv); }
