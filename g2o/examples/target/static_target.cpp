// This example consists of a single static target which sits in one
// place and does not move; in effect it has a "GPS" which measures
// its position

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>
 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/sampler.h>

#include "targetTypes3D.hpp"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main()
{
  // Set up the optimiser
  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // Create the block solver - the dimensions are specified because
  // 3D observations marginalise to a 3D estimate
  typedef BlockSolver<BlockSolverTraits<3, 3> > BlockSolver_3_3;
  BlockSolver_3_3::LinearSolverType* linearSolver
      = new LinearSolverCholmod<BlockSolver_3_3::PoseMatrixType>();
  BlockSolver_3_3* blockSolver
      = new BlockSolver_3_3(linearSolver);
  OptimizationAlgorithmGaussNewton* solver
    = new OptimizationAlgorithmGaussNewton(blockSolver);
  optimizer.setAlgorithm(solver);

  // Sample the actual location of the target
  Vector3d truePoint(sampleUniform(-500, 500),
                     sampleUniform(-500, 500),
                     sampleUniform(-500, 500));

  // Construct vertex which corresponds to the actual point of the target
  VertexPosition3D* position = new VertexPosition3D();
  position->setId(0);
  optimizer.addVertex(position);

  // Now generate some noise corrupted measurements; for simplicity
  // these are uniformly distributed about the true target. These are
  // modelled as a unary edge because they do not like to, say,
  // another node in the map.
  int numMeasurements = 10;
  double noiseLimit = sqrt(12.);
  double noiseSigma = noiseLimit*noiseLimit / 12.0;

  for (int i = 0; i < numMeasurements; i++)
    {
      Vector3d measurement = truePoint +
        Vector3d(sampleUniform(-0.5, 0.5) * noiseLimit,
                 sampleUniform(-0.5, 0.5) * noiseLimit,
                 sampleUniform(-0.5, 0.5) * noiseLimit);
      GPSObservationPosition3DEdge* goe = new GPSObservationPosition3DEdge();
      goe->setVertex(0, position);
      goe->setMeasurement(measurement);
      goe->setInformation(Matrix3d::Identity() / noiseSigma);
      optimizer.addEdge(goe);
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);
  
  cout << "truePoint=\n" << truePoint << endl;

  cerr <<  "computed estimate=\n" << dynamic_cast<VertexPosition3D*>(optimizer.vertices().find(0)->second)->estimate() << endl;

  //position->setMarginalized(true);
  
  SparseBlockMatrix<MatrixXd> spinv;

  optimizer.computeMarginals(spinv, position);



  //optimizer.solver()->computeMarginals();

  // covariance
  //
  cout << "covariance\n" << spinv << endl;

  cout << spinv.block(0,0) << endl;
  
}
