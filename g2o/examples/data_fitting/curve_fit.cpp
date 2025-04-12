// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
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
#include <iostream>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/sampler.h"

using namespace std;

G2O_USE_OPTIMIZATION_LIBRARY(dense);

/**
 * \brief the params, a, b, and lambda for a * exp(-lambda * t) + b
 */
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexParams() {}

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

  void setToOriginImpl() override {}

  void oplusImpl(const double* update) override {
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
  }
};

/**
 * \brief measurement for a point on the curve
 *
 * Here the measurement is the point which is lies on the curve.
 * The error function computes the difference between the curve
 * and the point.
 */
class EdgePointOnCurve
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointOnCurve() {}
  bool read(std::istream& /*is*/) override {
    G2O_ERROR("not implemented yet");
    return false;
  }
  bool write(std::ostream& /*os*/) const override {
    G2O_ERROR("not implemented yet");
    return false;
  }

  template <typename T>
  bool operator()(const T* params, T* error) const {
    const T& a = params[0];
    const T& b = params[1];
    const T& lambda = params[2];
    T fval = a * exp(-lambda * T(measurement()(0))) + b;
    error[0] = fval - measurement()(1);
    return true;
  }

  G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
};

int main(int argc, char** argv) {
  int numPoints;
  int maxIterations;
  bool verbose;
  string dumpFilename;
  g2o::CommandArgs arg;
  arg.param("dump", dumpFilename, "", "dump the points into a file");
  arg.param("numPoints", numPoints, 50,
            "number of points sampled from the curve");
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");

  arg.parseArgs(argc, argv);

  // generate random data
  g2o::Sampler::seedRand();
  double a = 2.;
  double b = 0.4;
  double lambda = 0.2;
  Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];
  for (int i = 0; i < numPoints; ++i) {
    double x = g2o::Sampler::uniformRand(0, 10);
    double y = a * exp(-lambda * x) + b;
    // add Gaussian noise
    y += g2o::Sampler::gaussRand(0, 0.02);
    points[i].x() = x;
    points[i].y() = y;
  }

  if (dumpFilename.size() > 0) {
    ofstream fout(dumpFilename.c_str());
    for (int i = 0; i < numPoints; ++i) fout << points[i].transpose() << endl;
  }

  // setup the solver
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // allocate the solver
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense",
                                                               solverProperty));

  // build the optimization problem given the points
  // 1. add the parameter vertex
  VertexParams* params = new VertexParams();
  params->setId(0);
  params->setEstimate(
      Eigen::Vector3d(1, 1, 1));  // some initial value for the params
  optimizer.addVertex(params);
  // 2. add the points we measured to be on the curve
  for (int i = 0; i < numPoints; ++i) {
    EdgePointOnCurve* e = new EdgePointOnCurve;
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, params);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  // perform the optimization
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  if (verbose) cout << endl;

  // print out the result
  cout << "Target curve" << endl;
  cout << "a * exp(-lambda * x) + b" << endl;
  cout << "Iterative least squares solution" << endl;
  cout << "a      = " << params->estimate()(0) << endl;
  cout << "b      = " << params->estimate()(1) << endl;
  cout << "lambda = " << params->estimate()(2) << endl;
  cout << endl;

  // clean up
  delete[] points;

  return 0;
}
