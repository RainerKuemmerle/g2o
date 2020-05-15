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

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;

double errorOfSolution(int numPoints, Eigen::Vector2d* points, const Eigen::Vector3d& circle)
{
  Eigen::Vector2d center = circle.head<2>();
  double radius = circle(2);
  double error = 0.;
  for (int i = 0; i < numPoints; ++i) {
    double d = (points[i] - center).norm() - radius;
    error += d*d;
  }
  return error;
}

/**
 * \brief a circle located at x,y with radius r
 */
class VertexCircle : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCircle()
    {
    }

    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual void setToOriginImpl()
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update)
    {
      Eigen::Vector3d::ConstMapType v(update);
      _estimate += v;
    }
};

/**
 * \brief measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCircle()
    {
    }
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    void computeError()
    {
      const VertexCircle* circle = static_cast<const VertexCircle*>(vertex(0));

      const Eigen::Vector2d& center = circle->estimate().head<2>();
      const double& radius = circle->estimate()(2);

      _error(0) = (measurement() - center).norm() - radius;
    }
};

int main(int argc, char** argv)
{
  int numPoints;
  int maxIterations;
  bool verbose;
  std::vector<int> gaugeList;
  g2o::CommandArgs arg;
  arg.param("numPoints", numPoints, 100, "number of points sampled from the circle");
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");

  arg.parseArgs(argc, argv);

  // generate random data
  Eigen::Vector2d center(4.0, 2.0);
  double radius = 2.0;
  Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];

  g2o::Sampler::seedRand();
  for (int i = 0; i < numPoints; ++i) {
    double r = g2o::Sampler::gaussRand(radius, 0.05);
    double angle = g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
    points[i].x() = center.x() + r * cos(angle);
    points[i].y() = center.y() + r * sin(angle);
  }

  // some handy typedefs
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
  typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;

  // setup the solver
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  optimizer.setAlgorithm(solver);

  // build the optimization problem given the points
  // 1. add the circle vertex
  VertexCircle* circle = new VertexCircle();
  circle->setId(0);
  circle->setEstimate(Eigen::Vector3d(3.0, 3.0, 3.0)); // some initial value for the circle
  optimizer.addVertex(circle);
  // 2. add the points we measured
  for (int i = 0; i < numPoints; ++i) {
    EdgePointOnCircle* e = new EdgePointOnCircle;
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, circle);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  // perform the optimization
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  if (verbose)
    cout << endl;

  // print out the result
  cout << "Iterative least squares solution" << endl;
  cout << "center of the circle " << circle->estimate().head<2>().transpose() << endl;
  cout << "radius of the cirlce " << circle->estimate()(2) << endl;
  cout << "error " << errorOfSolution(numPoints, points, circle->estimate()) << endl;
  cout << endl;

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
    A(i, 0) = -2*points[i].x();
    A(i, 1) = -2*points[i].y();
    A(i, 2) = 1;
    b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
  }
  Eigen::Vector3d solution = (A.transpose()*A).ldlt().solve(A.transpose() * b);
  // calculate the radius of the circle given the solution so far
  solution(2) = sqrt(pow(solution(0), 2) + pow(solution(1), 2) - solution(2));
  cout << "Linear least squares solution" << endl;
  cout << "center of the circle " << solution.head<2>().transpose() << endl;
  cout << "radius of the cirlce " << solution(2) << endl;
  cout << "error " << errorOfSolution(numPoints, points, solution) << endl;

  // clean up
  delete[] points;

  return 0;
}
