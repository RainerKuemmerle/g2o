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

#ifndef G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H
#define G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H

#include "optimization_algorithm_with_hessian.h"

namespace g2o {

  /**
   * \brief Implementation of the Gauss Newton Algorithm
   */
  class OptimizationAlgorithmGaussNewton : public OptimizationAlgorithmWithHessian
  {
    public:
      /**
       * construct the Gauss Newton algorithm, which use the given Solver for solving the
       * linearized system.
       */
      explicit OptimizationAlgorithmGaussNewton(Solver* solver);
      virtual ~OptimizationAlgorithmGaussNewton();

      virtual SolverResult solve(int iteration, bool online = false);

      virtual void printVerbose(std::ostream& os) const;
  };

} // end namespace

#endif
