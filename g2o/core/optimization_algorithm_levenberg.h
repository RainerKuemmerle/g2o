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

#ifndef G2O_SOLVER_LEVENBERG_H
#define G2O_SOLVER_LEVENBERG_H

#include "optimization_algorithm_with_hessian.h"

namespace g2o {

  /**
   * \brief Implementation of the Gauss Newton Algorithm
   */
  class OptimizationAlgorithmLevenberg : public OptimizationAlgorithmWithHessian
  {
    public:
      /**
       * construct the Levenberg algorithm, which will use the given Solver for solving the
       * linearized system.
       */
      explicit OptimizationAlgorithmLevenberg(Solver* solver);
      virtual ~OptimizationAlgorithmLevenberg();

      virtual SolverResult solve(int iteration, bool online = false);

      virtual void printVerbose(std::ostream& os) const;

      //! return the currently used damping factor
      double currentLambda() const { return _currentLambda;}

      //! the number of internal iteration if an update step increases chi^2 within Levenberg-Marquardt
      void setMaxTrialsAfterFailure(int max_trials);

      //! get the number of inner iterations for Levenberg-Marquardt
      int maxTrialsAfterFailure() const { return _maxTrialsAfterFailure->value();}

      //! return the lambda set by the user, if < 0 the SparseOptimizer will compute the initial lambda
      double userLambdaInit() {return _userLambdaInit->value();}
      //! specify the initial lambda used for the first iteraion, if not given the SparseOptimizer tries to compute a suitable value
      void setUserLambdaInit(double lambda);

    protected:
      // Levenberg parameters
      Property<int>* _maxTrialsAfterFailure;
      Property<double>* _userLambdaInit;
      double _currentLambda;
      double _tau;
      double _goodStepLowerScale; ///< lower bound for lambda decrease if a good LM step
      double _goodStepUpperScale; ///< upper bound for lambda decrease if a good LM step
      double _ni;

      /**
       * helper for Levenberg, this function computes the initial damping factor, if the user did not
       * specify an own value, see setUserLambdaInit()
       */
      double computeLambdaInit() const;
      double computeScale() const;

  };

} // end namespace

#endif
