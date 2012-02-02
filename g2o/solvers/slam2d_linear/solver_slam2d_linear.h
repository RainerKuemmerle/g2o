// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_SOLVER_SLAM2D_LINEAR
#define G2O_SOLVER_SLAM2D_LINEAR

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o_slam2d_linear_api.h"

namespace g2o {

  class SparseOptimizer;
  class Solver;

  /**
   * \brief Implementation of a linear approximation for 2D pose graph SLAM
   *
   * Needs to operate on the full graph, whereas the nodes connected by
   * odometry are 0 -> 1 -> 2 -> ...
   * Furthermore excactly one node should be the fixed vertex.
   * Within the first iteration the orientation of the nodes is computed. In
   * the subsequent iterations full non-linear GN is carried out.
   * The linear approximation is correct, if the covariance of the constraints
   * is a diagonal matrix.
   *
   * More or less the solver is an implementation of the approach described
   * by Carlone et al, RSS'11.
   */
  class G2O_SLAM2D_LINEAR_API SolverSLAM2DLinear : public OptimizationAlgorithmGaussNewton
  {
    public:
      /**
       * Construct a Solver for solving 2D pose graphs. Within the first iteration
       * the rotations are solved and afterwards standard non-linear Gauss Newton
       * is carried out.
       */
      explicit SolverSLAM2DLinear(Solver* solver);
      virtual ~SolverSLAM2DLinear();

      virtual OptimizationAlgorithm::SolverResult solve(int iteration, bool online = false);

    protected:
      bool solveOrientation();
  };

} // end namespace

#endif
