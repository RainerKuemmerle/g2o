// This example illustrates how to use a dynamic vertex in a graph.

// The goal is to fit a polynomial y=p(x) to a set of data. The degree of the
// polynomial is user-defined.

#include <unsupported/Eigen/Polynomials>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

// Declare the custom types used in the graph

// This vertex stores the coefficients of the polynomial. It is dynamic because
// we can change it at runtime.

class PolynomialCoefficientVertex : public g2o::BaseVertex<Eigen::Dynamic, Eigen::VectorXd>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Create the vertex and assign the starting dimension
  PolynomialCoefficientVertex(int startingDimension = 0) {
    setEstimateDimension(startingDimension);
  }

  // Read the vertex
  virtual bool read(std::istream& is)
  {
    // Read the dimension
    int dimension;
    is >> dimension;
    if (is.good() == false) {
      return false;
    }

    // Resize; call the resize method because this ensures that the Jacobian
    // workspace is properly updated
    setEstimateDimension(dimension);

    // Read the state
    return g2o::internal::readVector(is, _estimate);
   }

  // Write the vertex
  virtual bool write(std::ostream& os) const
  {
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
    //std::cout << v << std::endl;
    _estimate += v;
  }

  // Resize the vertex state. This uses Eigen's conservative resize.
  // Note we do not zero out the vector to keep what coefficients can be
  // kept which changing size.
  virtual bool setEstimateDimensionImpl(int newDimension)
  {
    int oldDimension = estimateDimension();

    // Handle the special case this is the first time
    //if (_dimension == Eigen::Dynamic) {
      std::cout << "*" << std::endl;
      _estimate.resize(newDimension);
      _estimate.setZero();
      return true;
      //}

    _estimate.conservativeResize(newDimension);

    // If the state has expanded, set the new values to zero
    std::cout << oldDimension << ":" << newDimension << std::endl;
    if (oldDimension < newDimension)
      _estimate.tail(newDimension-oldDimension).setZero();

    return true;
  }
};

// This edge provides an observation of the polynomial at a single value.
// The assumed model is z = p(x) + w,
// where w is the additive noise with covariance equal to Omega.

// Note that x is not a measurement so it has to be stored separately.

class PolynomialSingleValueEdge : public g2o::BaseUnaryEdge<1, double, PolynomialCoefficientVertex>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PolynomialSingleValueEdge(double x, double z, const PolynomialSingleValueEdge::InformationType& omega)
  {
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

  // Set ground truth dimensions
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

  // Set number of observations
  int obs = 6;
  if (argc > 2) {
    obs = atoi(argv[2]);
  }  

  // Sample the observations
  double sigmaZ = 0.1;
  Eigen::VectorXd x(obs);
  Eigen::VectorXd z(obs);

  for (int i = 0; i < obs; ++i) {
    x[i] = g2o::sampleUniform(-5, 5);
    z[i] = Eigen::poly_eval(p, x[i]) + sigmaZ * g2o::sampleGaussian();
   }

  // Construct the graph and set up the solver and optimiser
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver = g2o::make_unique<
    g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

  // Set up the solver
  std::unique_ptr<g2o::BlockSolverX> blockSolver = g2o::make_unique<g2o::BlockSolverX>(
      move(linearSolver));

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
  
  // Create the edges
  PolynomialSingleValueEdge* pe;
  for (int i = 0; i < obs; ++i)
    {
      pe = new PolynomialSingleValueEdge(x[i], z[i], omega);
      pe->setVertex(0, pv);
      optimizer->addEdge(pe);
    }

  // Iterate over different vertex sizes from 1 to the number of dimensions
  for (int testDimension = 1; testDimension <= polynomialDimension; ++testDimension) {
    pv->setEstimateDimension(testDimension);
    optimizer->initializeOptimization();
    optimizer->optimize(10);
    std::cout << "Computed parameters = " << pv->estimate().transpose() << std::endl;
  }
  
  /*
  Eigen::VectorXd x(5);
  x[0] = -2;
  x[1] = -1;
  x[2] = 0;
  x[3] = 1;
  x[4] = 2;

  for (int i = 0; i < 5; ++i) {
    std::cout << x[i] << ":" << Eigen::poly_eval(p, x[i]) << std::endl;
    }*/
}
