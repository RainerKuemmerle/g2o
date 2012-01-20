// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
