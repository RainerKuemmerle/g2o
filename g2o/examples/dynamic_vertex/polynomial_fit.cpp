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
    resizeDimension(startingDimension);
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
    resizeDimension(dimension);

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
    _estimate += v;
  }

  // Resize the vertex state. This uses Eigen's conservative resize.
  // Note we do not zero out the vector to keep what coefficients can be
  // kept which changing size.
  virtual bool resizeDimensionImpl(int newDimension)
  {
    int oldDimension = _estimate.size();

    // This attempts to keep as much of the state as possible
    _estimate.conservativeResize(newDimension);
    
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

  // Quick test

  Eigen::VectorXd p(3);

  p[0] = 1;
  p[1] = 1;
  p[2] = 0;


  Eigen::VectorXd x(5);
  x[0] = -2;
  x[1] = -1;
  x[2] = 0;
  x[3] = 1;
  x[4] = 1;

  for (int i = 0; i < 5; ++i) {
    std::cout << Eigen::poly_eval(p, x[i]) << std::endl;
  }
  PolynomialCoefficientVertex* pv = new PolynomialCoefficientVertex();

  std::cout << pv->estimate() << std::endl;

  pv->resizeDimension(2);
  std::cout << pv->estimate() << std::endl;
  
}
