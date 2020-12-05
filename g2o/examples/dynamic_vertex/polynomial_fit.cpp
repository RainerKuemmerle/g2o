// This example illustrates how to use a dynamic vertex in a graph.

// The goal is to fit a polynomial y(x)=p(x) to a set of data. The degree of the
// polynomial is user-defined. The amount of samples is user defined as well.

// Each observation consists of the pair Z_i=(x_i,z_i) where
// z_i=y(x_i)+w_i, where w_i is additive white noise with information
// matrix Omega.

#include <unsupported/Eigen/Polynomials>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_dynamic_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

// Declare the custom types used in the graph

// This vertex stores the coefficients of the polynomial. It is dynamic because
// we can change it at runtime.

class PolynomialCoefficientVertex : public g2o::BaseDynamicVertex<Eigen::VectorXd> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Create the vertex
  PolynomialCoefficientVertex() {
  }

  // Read the vertex
  virtual bool read(std::istream& is) {
    // Read the dimension
    int dimension;
    is >> dimension;
    if (is.good() == false) {
      return false;
    }

    // Set the dimension; we call the method here to ensure stuff like
    // cache and the workspace is setup
    setDimension(dimension);

    // Read the state
    return g2o::internal::readVector(is, _estimate);
  }

  // Write the vertex
  virtual bool write(std::ostream& os) const {
    os << _estimate.size() << " ";
    return g2o::internal::writeVector(os, _estimate);
  }

  // Reset to zero
  virtual void setToOriginImpl() {
    _estimate.setZero();
  }

  // Direct linear add
  virtual void oplusImpl(const double* update) {
    Eigen::VectorXd::ConstMapType v(update, _dimension);
    _estimate += v;
  }

  // Resize the vertex state. In this case, we want to preserve as much of the
  // state as we can. Therefore, we use conservativeResize and pad with zeros
  // at the end if the state dimension has increased.
  virtual bool setDimensionImpl(int newDimension)
  {
    int oldDimension = dimension();

    // Handle the special case this is the first time
    if (oldDimension == Eigen::Dynamic) {
      _estimate.resize(newDimension);
      _estimate.setZero();
      return true;
    }

    _estimate.conservativeResize(newDimension);

    // If the state has expanded, pad with zeros
    if (oldDimension < newDimension)
      _estimate.tail(newDimension-oldDimension).setZero();

    return true;
  }
};

// This edge provides an observation of the polynomial at a single value.
// The assumed model is z = p(x) + w,
// where w is the additive noise with covariance equal to inv(Omega).

// Note that x is not a measurement so it has to be stored separately.

class PolynomialSingleValueEdge : public g2o::BaseUnaryEdge<1, double, PolynomialCoefficientVertex> {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PolynomialSingleValueEdge(double x, double z, const PolynomialSingleValueEdge::InformationType& omega) : _x(x) {
    _x = x;
    setMeasurement(z);
    setInformation(omega);
  }

  virtual bool read(std::istream& is) {
    double z;
    is >> _x >> z;
    setMeasurement(z);
    return readInformationMatrix(is);
  }

  virtual bool write(std::ostream& os) const {
    os << _x << " " << _measurement;
    return writeInformationMatrix(os);
  }

  // Compute the measurement from the eigen polynomial module
  virtual void computeError() {
    const PolynomialCoefficientVertex* vertex = static_cast<const PolynomialCoefficientVertex*> (_vertices[0]);
    _error[0] = _measurement - Eigen::poly_eval(vertex->estimate(), _x);
  }

private:

  // The point that the polynomial is computed at
  double _x;
};

int main(int argc, const char* argv[]) {

  // Number of dimensions of the polynomial; the default is 4
  int polynomialDimension = 4;
  if (argc > 1) {
    polynomialDimension = atoi(argv[1]);
  }

  // Create the coefficients for the polynomial (all drawn randomly)
  Eigen::VectorXd p(polynomialDimension);
  for (int i = 0; i < polynomialDimension; ++i) {
    p[i] = g2o::sampleUniform(-1, 1);
  }

  std::cout << "Ground truth vector=" << p.transpose() << std::endl;

  // The number of observations in the polynomial; the defautl is 6
  int obs = 6;
  if (argc > 2) {
    obs = atoi(argv[2]);
  }

  // Sample the observations; we don't do anything with them here, but
  // they could be plotted
  double sigmaZ = 0.1;
  Eigen::VectorXd x(obs);
  Eigen::VectorXd z(obs);

  for (int i = 0; i < obs; ++i) {
    x[i] = g2o::sampleUniform(-5, 5);
    z[i] = Eigen::poly_eval(p, x[i]) + sigmaZ * g2o::sampleGaussian();
  }

  // Construct the graph and set up the solver and optimiser
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
      g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();

  // Set up the solver
  std::unique_ptr<g2o::BlockSolverX> blockSolver =
      g2o::make_unique<g2o::BlockSolverX>(move(linearSolver));

  // Set up the optimisation algorithm
  g2o::OptimizationAlgorithm* optimisationAlgorithm =
    new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

  // Create the graph and configure it
  std::unique_ptr<g2o::SparseOptimizer> optimizer = g2o::make_unique<g2o::SparseOptimizer>();
  optimizer->setVerbose(true);
  optimizer->setAlgorithm(optimisationAlgorithm);

  // Create the vertex; note its dimension is currently is undefined
  PolynomialCoefficientVertex* pv = new PolynomialCoefficientVertex();
  pv->setId(0);
  optimizer->addVertex(pv);

  // Create the information matrix
  PolynomialSingleValueEdge::InformationType omega = PolynomialSingleValueEdge::InformationType::Zero();
  omega(0, 0) = 1 / (sigmaZ * sigmaZ);

  // Create the observations and the edges
  for (int i = 0; i < obs; ++i) {
    PolynomialSingleValueEdge* pe = new PolynomialSingleValueEdge(x[i], z[i], omega);
    pe->setVertex(0, pv);
    optimizer->addEdge(pe);
  }

  // Now run the same optimization problem for different choices of
  // dimension of the polynomial vertex. This shows how we can
  // dynamically change the vertex dimensions in an alreacy
  // constructed graph. Note that you must call initializeOptimization
  // before you can optimize after a state dimension has changed.
  for (int testDimension = 1; testDimension <= polynomialDimension; ++testDimension) {
    pv->setDimension(testDimension);
    optimizer->initializeOptimization();
    optimizer->optimize(10);
    std::cout << "Computed parameters = " << pv->estimate().transpose() << std::endl;
  }
  for (int testDimension = polynomialDimension - 1; testDimension >= 1; --testDimension) {
    pv->setDimension(testDimension);
    optimizer->initializeOptimization();
    optimizer->optimize(10);
    std::cout << "Computed parameters = " << pv->estimate().transpose() << std::endl;
  }
}
