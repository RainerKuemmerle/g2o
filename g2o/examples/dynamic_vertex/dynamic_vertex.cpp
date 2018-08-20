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
    cout << __PRETTY_FUNCTION__ << endl;
    
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


class DynamicBinaryEdge : public g2o::BaseBinaryEdge<Eigen::Dynamic, Eigen::VectorXd, DynamicVertex, DynamicVertex>
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
    _error = static_cast<DynamicVertex*>(_vertices[0])->estimate().head(_dimension) - static_cast<DynamicVertex*>(_vertices[1])->estimate().head(_dimension) - _measurement.head(_dimension);
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
  optimizer->setVerbose(true);
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
  DynamicVertex* v0 = new DynamicVertex();
  v0->setId(0);
  
  v0->resizeDimension(dimension);
  v0->setToOrigin();

  optimizer->addVertex(v0);

  // Create a measurement
  VectorXd measurement(dimension);
  for (int d = 0; d < dimension; ++ d)
    measurement[d] = d;

  MatrixXd information(dimension, dimension);
  information.setIdentity();

  // Create the edge
  DynamicUnaryEdge* due = new DynamicUnaryEdge();
  due->setVertex(0, v0);
  
  due->setMeasurement(measurement);
  due->setInformation(information);

  optimizer->addEdge(due);

  optimizer->initializeOptimization();
  
  optimizer->optimize(10);

  cout << v0->estimate().transpose() << endl;

  delete optimizer;
}


// Test what happens if we resize a vertex with a unary edge
void testReizeUnaryEdge(int dim1, int dim2)
{
  // Create the optimizer
  g2o::SparseOptimizer* optimizer = buildOptimizer();

  // Create the vertex, set the dimensions and clear the state to set
  // it to a known value
  DynamicVertex* v0 = new DynamicVertex();
  v0->setId(0);
  
  v0->resizeDimension(dim1);
  v0->setToOrigin();

  optimizer->addVertex(v0);

  // Create a measurement
  VectorXd measurement(dim1);
  for (int d = 0; d < dim1; ++ d)
    measurement[d] = d;

  MatrixXd information(dim1, dim1);
  information.setIdentity();

  // Create the edge
  DynamicUnaryEdge* due = new DynamicUnaryEdge();
  due->setVertex(0, v0);
  
  due->setMeasurement(measurement);
  due->setInformation(information);

  optimizer->addEdge(due);

  optimizer->initializeOptimization();
  
  optimizer->optimize(10);

  cout << v0->estimate().transpose() << endl;

  // Now resize everything
  v0->resizeDimension(dim2);
  v0->setToOrigin();

  measurement.resize(dim2);
  for (int d = 0; d < dim2; ++ d)
    measurement[d] = d;

  information.resize(dim2, dim2);
  information.setIdentity();
  
  
  due->setMeasurement(measurement);
  due->setInformation(information);
  cout << due->rank() << endl;

  optimizer->initializeOptimization();
  optimizer->computeInitialGuess();
  
  optimizer->optimize(10);
  /*

  cout << v0->estimate().transpose() << endl;
  */
  delete optimizer;
}

void testBinaryEdgeDD(int dimV0, int dimV1, int dimM)
{
  // Create the optimizer
  g2o::SparseOptimizer* optimizer = buildOptimizer();

  // Create the vertex, set the dimensions and clear the state to set
  // it to a known value
  DynamicVertex* v0 = new DynamicVertex();
  v0->setId(0);
  v0->resizeDimension(dimV0);
  v0->setToOrigin();
  optimizer->addVertex(v0);

  DynamicVertex* v1 = new DynamicVertex();
  v1->setId(1);
  v1->resizeDimension(dimV1);
  v1->setToOrigin();

  optimizer->addVertex(v1);

    // Create a measurement
  VectorXd measurement(dimM);
  for (int d = 0; d < dimM; ++ d)
    measurement[d] = d;

  MatrixXd information(dimM, dimM);
  information.setIdentity();

  // Create the edge
  DynamicBinaryEdge* dbe = new DynamicBinaryEdge();
  dbe->setVertex(0, v0);
  dbe->setVertex(1, v1);
  
  dbe->setMeasurement(measurement);
  dbe->setInformation(information);

  optimizer->addEdge(dbe);

  optimizer->initializeOptimization();
  
  optimizer->optimize(10);

  cout << v0->estimate().transpose() << endl;
  cout << v1->estimate().transpose() << endl;

  delete optimizer;
}

int main(int argc, const char* argv[])
{
  // First create a dynamic vertex

  //StaticVertex* s2 = new StaticVertex();
  //cout << s2->dimension() << endl;

  //testUnaryEdge(1);
  // testUnaryEdge(2);
  //testUnaryEdge(20);

  testReizeUnaryEdge(1, 2);
  
    
    
    //testBinaryEdgeDD(5, 5, 5);
    //testBinaryEdgeDD(10, 5, 5);
    //testBinaryEdgeDD(5, 10, 5);
    
  /*
  
  

  cout << "due->rank()=" << due->rank() << endl;
  
  
  due->computeError();

  cout << due->error().transpose() << endl;
  cout << "due->rank()=" << due->rank() << endl;

  v0->resizeDimension(12);
  measurement.resize(12);
  measurement.setZero();

  due->setMeasurement(measurement);
  
  due->computeError();

  cout << due->error().transpose() << endl;
  */ 
}
