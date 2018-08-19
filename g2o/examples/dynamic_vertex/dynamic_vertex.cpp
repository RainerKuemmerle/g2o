#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

/*
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <Eigen/Eigenvalues>
*/

using namespace ::std;
using namespace ::Eigen;

// A dynamic vertex implementation using a vector representation as
// the underlying type.

class DynamicVertex : public g2o::BaseVertex<Eigen::Dynamic, Eigen::VectorXd>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  DynamicVertex()
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
    _estimate.setZero();
  }
  
  virtual void oplusImpl(const double* update)
  {
    Eigen::VectorXd::ConstMapType v(update, _dimension);
      _estimate += v;
  }

  virtual void resizeDimensionImpl(int newDimension)
  {
     _estimate.resize(newDimension);
     _estimate.setZero();
   }
};

class StaticVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  StaticVertex()
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
    _estimate.setZero();
  }
  
  virtual void oplusImpl(const double* update)
  {
    Eigen::VectorXd::ConstMapType v(update, _dimension);
      _estimate += v;
  }
};

class DynamicUnaryEdge : public g2o::BaseUnaryEdge<Eigen::Dynamic, Eigen::VectorXd, DynamicVertex>
{
public:

  
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
  
  virtual void computeError()
  {
    _error = _measurement - static_cast<DynamicVertex*>(_vertices[0])->estimate();
  }
  
};

g2o::SparseOptimizer* buildOptimizer()
{
  unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver = g2o::make_unique<
    g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

  // Set up the solver
  unique_ptr<g2o::BlockSolverX> blockSolver = g2o::make_unique<g2o::BlockSolverX>(
      move(linearSolver));

  // Set up the optimisation algorithm
  g2o::OptimizationAlgorithm* optimisationAlgorithm =
    new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

  // Create the graph and configure it
  g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
  optimizer->setAlgorithm(optimisationAlgorithm);

  return optimizer;
}


// Test a unary edge of a given dimension.
void testUnaryEdge(int dimension)
{
  // Create the optimizer
  g2o::SparseOptimizer* optimizer = buildOptimizer();

  // Create the vertex, set the dimensions and clear the state to set
  // it to a known value
  DynamicVertex* v1 = new DynamicVertex();
  v1->setId(0);
  
  v1->resizeDimension(dimension);
  v1->setToOrigin();

  optimizer->addVertex(v1);

  // Create a measurement
  VectorXd measurement(dimension);
  measurement.setZero();

  MatrixXd information(dimension, dimension);
  information.setIdentity();

  
  delete optimizer;
}


int main(int argc, const char* argv[])
{
  // First create a dynamic vertex

  //StaticVertex* s2 = new StaticVertex();
  //cout << s2->dimension() << endl;

    testUnaryEdge(1);

  /*
  
  DynamicUnaryEdge* due = new DynamicUnaryEdge();
  due->setVertex(0, v1);

  
  due->setMeasurement(measurement);
  due->setInformation(information);
  

  cout << "due->rank()=" << due->rank() << endl;
  
  
  due->computeError();

  cout << due->error().transpose() << endl;
  cout << "due->rank()=" << due->rank() << endl;

  v1->resizeDimension(12);
  measurement.resize(12);
  measurement.setZero();

  due->setMeasurement(measurement);
  
  due->computeError();

  cout << due->error().transpose() << endl;
  */ 
}
