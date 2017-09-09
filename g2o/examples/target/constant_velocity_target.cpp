// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

// This example consists of a single constant velocity target which
// moves under piecewise constant velocity in 3D. Its position is
// measured by an idealised GPS receiver.

#include <Eigen/StdVector>
#include <iostream>

#include <stdint.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/sampler.h>

#include "targetTypes6D.hpp"
#include "continuous_to_discrete.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main()
{
  // Set up the parameters of the simulation
  int numberOfTimeSteps = 1000;
  const double processNoiseSigma = 1;
  const double accelerometerNoiseSigma = 1;
  const double gpsNoiseSigma = 1;
  const double dt = 1;  

  // Set up the optimiser and block solver
  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  typedef BlockSolver< BlockSolverTraits<6, 6> > BlockSolver;

  OptimizationAlgorithm* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolver>(
      g2o::make_unique<LinearSolverCholmod<BlockSolver::PoseMatrixType>>()));

  optimizer.setAlgorithm(optimizationAlgorithm);

  // Sample the start location of the target
  Vector6d state;
  state.setZero();
  for (int k = 0; k < 3; k++)
    {
      state[k] = 1000 * sampleGaussian();
    }
  
  // Construct the first vertex; this corresponds to the initial
  // condition and register it with the optimiser
  VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
  stateNode->setEstimate(state);
  stateNode->setId(0);
  optimizer.addVertex(stateNode);

  // Set up last estimate
  VertexPositionVelocity3D* lastStateNode = stateNode;

  // Iterate over the simulation steps
  for (int k = 1; k <= numberOfTimeSteps; ++k)
    {
      // Simulate the next step; update the state and compute the observation
      Vector3d processNoise(processNoiseSigma*sampleGaussian(),
                            processNoiseSigma*sampleGaussian(),
                            processNoiseSigma*sampleGaussian());

      for (int m = 0; m < 3; m++)
        {
          state[m] += dt * (state[m+3] + 0.5 * dt * processNoise[m]);
        }

      for (int m = 0; m < 3; m++)
        {
          state[m+3] += dt * processNoise[m];
        }

      // Construct the accelerometer measurement
      Vector3d accelerometerMeasurement;
      for (int m = 0; m < 3; m++)
        {
          accelerometerMeasurement[m] = processNoise[m] + accelerometerNoiseSigma * sampleGaussian();
        }

      // Construct the GPS observation
      Vector3d gpsMeasurement;     
      for (int m = 0; m < 3; m++)
        {
          gpsMeasurement[m] = state[m] + gpsNoiseSigma * sampleGaussian();
        }

      // Construct vertex which corresponds to the current state of the target
      VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
      
      stateNode->setId(k);
      stateNode->setMarginalized(false);
      optimizer.addVertex(stateNode);

      TargetOdometry3DEdge* toe = new TargetOdometry3DEdge(dt, accelerometerNoiseSigma);
      toe->setVertex(0, lastStateNode);
      toe->setVertex(1, stateNode);
      VertexPositionVelocity3D* vPrev= dynamic_cast<VertexPositionVelocity3D*>(lastStateNode);
      VertexPositionVelocity3D* vCurr= dynamic_cast<VertexPositionVelocity3D*>(stateNode);
      toe->setMeasurement(accelerometerMeasurement);
      optimizer.addEdge(toe);
      
      // compute the initial guess via the odometry
      g2o::OptimizableGraph::VertexSet vPrevSet;
      vPrevSet.insert(vPrev);
      toe->initialEstimate(vPrevSet,vCurr);

      lastStateNode = stateNode;

      // Add the GPS observation
      GPSObservationEdgePositionVelocity3D* goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
      goe->setVertex(0, stateNode);
      optimizer.addEdge(goe);
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);
  cerr << "number of vertices:" << optimizer.vertices().size() << endl;
  cerr << "number of edges:" << optimizer.edges().size() << endl;

  // Print the results

  cout << "state=\n" << state << endl;

#if 0
  for (int k = 0; k < numberOfTimeSteps; k++)
    {
      cout << "computed estimate " << k << "\n"
           << dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(k)->second)->estimate() << endl;
       }
#endif

  Vector6d v1 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(numberOfTimeSteps-2,0))->second)->estimate();
  Vector6d v2 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(numberOfTimeSteps-1,0))->second)->estimate();
  cout << "v1=\n" << v1 << endl;
  cout << "v2=\n" << v2 << endl;
  cout << "delta state=\n" << v2-v1 << endl;
}
