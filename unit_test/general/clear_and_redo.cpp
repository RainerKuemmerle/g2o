#include "gtest/gtest.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

TEST(General, ClearAndRedo)
{   
  // Initialize the SparseOptimizer
  g2o::SparseOptimizer mOptimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  mOptimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));

  // Set the default terminate action
  g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
  mOptimizer.addPostIterationAction(terminateAction);

  for(int i = 0; i < 2; i++)
  {
    // Add vertices
    g2o::VertexSE3* v0 = new g2o::VertexSE3;
    v0->setEstimate(Eigen::Transform<double,3,1>(Eigen::Translation<double, 3>(0,0,0)));
    v0->setId(0);
    mOptimizer.addVertex(v0);

    g2o::VertexSE3* v1 = new g2o::VertexSE3;
    v1->setEstimate(Eigen::Transform<double,3,1>(Eigen::Translation<double, 3>(0,0,0)));
    v1->setId(1);
    mOptimizer.addVertex(v1);

    g2o::VertexSE3* v2 = new g2o::VertexSE3;
    v2->setEstimate(Eigen::Transform<double,3,1>(Eigen::Translation<double, 3>(0,0,0)));
    v2->setId(2);
    mOptimizer.addVertex(v2);

    // Add edges
    g2o::EdgeSE3* e1 = new g2o::EdgeSE3();
    e1->vertices()[0] = mOptimizer.vertex(0);
    e1->vertices()[1] = mOptimizer.vertex(1);
    e1->setMeasurement(Eigen::Isometry3d(Eigen::Translation<double, 3>(1,0,0)));
    e1->setInformation(Eigen::Matrix<double,6,6>::Identity());
    mOptimizer.addEdge(e1);

    g2o::EdgeSE3* e2 = new g2o::EdgeSE3();
    e2->vertices()[0] = mOptimizer.vertex(1);
    e2->vertices()[1] = mOptimizer.vertex(2);
    e2->setMeasurement(Eigen::Isometry3d(Eigen::Translation<double, 3>(0,1,0)));
    e2->setInformation(Eigen::Matrix<double,6,6>::Identity());
    mOptimizer.addEdge(e2);

    g2o::EdgeSE3* e3 = new g2o::EdgeSE3();
    e3->vertices()[0] = mOptimizer.vertex(2);
    e3->vertices()[1] = mOptimizer.vertex(0);
    e3->setMeasurement(Eigen::Isometry3d(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1)));
    e3->setInformation(Eigen::Matrix<double,6,6>::Identity());
    mOptimizer.addEdge(e3);

    v0->setFixed(true);

    //mOptimizer.setVerbose(true);
    mOptimizer.initializeOptimization();
    mOptimizer.computeInitialGuess();
    mOptimizer.computeActiveErrors();
    int iter = mOptimizer.optimize(10);
    if (iter <= 0)
    {
      ADD_FAILURE();
    } else {
      SUCCEED();
    }

    mOptimizer.clear();
  }
}
