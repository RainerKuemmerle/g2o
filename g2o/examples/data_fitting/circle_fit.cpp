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
#include <Eigen/Geometry>
#include <iostream>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"

G2O_USE_OPTIMIZATION_LIBRARY(dense);

namespace {
using PointVector =
    std::vector<g2o::Vector2, Eigen::aligned_allocator<g2o::Vector2>>;

double errorOfSolution(const PointVector& points,
                       const Eigen::Vector3d& circle) {
  const Eigen::Vector2d center = circle.head<2>();
  const double radius = circle(2);
  double error = 0.;
  for (const auto& point : points) {
    const double distance = (point - center).norm() - radius;
    error += distance * distance;
  }
  return error;
}

}  // namespace

/**
 * \brief a circle located at x,y with radius r
 */
class VertexCircle : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

  void setToOriginImpl() override {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  }

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update.head<kDimension>();
  }
};

/**
 * \brief measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool read(std::istream& /*is*/) override { return false; }
  bool write(std::ostream& /*os*/) const override { return false; }

  template <typename T>
  bool operator()(const T* circle, T* error) const {
    typename g2o::VectorN<2, T>::ConstMapType center(circle);
    const T& radius = circle[2];

    error[0] = (measurement().cast<T>() - center).norm() - radius;
    return true;
  }

  G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
};

int main(int argc, char** argv) {
  int numPoints;
  int maxIterations;
  bool verbose;
  g2o::CommandArgs arg;
  arg.param("numPoints", numPoints, 100,
            "number of points sampled from the circle");
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.parseArgs(argc, argv);

  // generate random data
  const Eigen::Vector2d center(4.0, 2.0);
  constexpr double kRadius = 2.0;
  PointVector points(numPoints);

  g2o::Sampler::seedRand();
  for (int i = 0; i < numPoints; ++i) {
    const double r = g2o::Sampler::gaussRand(kRadius, 0.05);
    const double angle = g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
    points[i].x() = center.x() + r * cos(angle);
    points[i].y() = center.y() + r * sin(angle);
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
  // 1. add the circle vertex
  auto circle = std::make_shared<VertexCircle>();
  circle->setId(0);
  circle->setEstimate(
      Eigen::Vector3d(3.0, 3.0, 3.0));  // some initial value for the circle
  optimizer.addVertex(circle);
  // 2. add the points we measured
  for (int i = 0; i < numPoints; ++i) {
    auto e = std::make_shared<EdgePointOnCircle>();
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, circle);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  // perform the optimization
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  if (verbose) std::cout << std::endl;

  // print out the result
  std::cout << "Iterative least squares solution" << std::endl;
  std::cout << "center of the circle "
            << circle->estimate().head<2>().transpose() << std::endl;
  std::cout << "radius of the circle " << circle->estimate()(2) << std::endl;
  std::cout << "error " << errorOfSolution(points, circle->estimate())
            << std::endl;
  std::cout << std::endl;

  // solve by linear least squares
  // Let (a, b) be the center of the circle and r the radius of the circle.
  // For a point (x, y) on the circle we have:
  // (x - a)^2 + (y - b)^2 = r^2
  // This leads to
  // (-2x -2y 1)^T * (a b c) = -x^2 - y^2   (1)
  // where c = a^2 + b^2 - r^2.
  // Since we have a bunch of points, we accumulate Eqn (1) in a matrix and
  // compute the normal equation to obtain a solution for (a b c).
  // Afterwards the radius r is recovered.
  Eigen::MatrixXd A(numPoints, 3);
  Eigen::VectorXd b(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    A(i, 0) = -2 * points[i].x();
    A(i, 1) = -2 * points[i].y();
    A(i, 2) = 1;
    b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
  }
  Eigen::Vector3d solution =
      (A.transpose() * A).ldlt().solve(A.transpose() * b);
  // calculate the radius of the circle given the solution so far
  solution(2) = sqrt(pow(solution(0), 2) + pow(solution(1), 2) - solution(2));
  std::cout << "Linear least squares solution" << std::endl;
  std::cout << "center of the circle " << solution.head<2>().transpose()
            << std::endl;
  std::cout << "radius of the circle " << solution(2) << std::endl;
  std::cout << "error " << errorOfSolution(points, solution) << std::endl;

  return 0;
}
