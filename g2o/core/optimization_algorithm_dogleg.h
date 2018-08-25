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

#ifndef G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H
#define G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H

#include "optimization_algorithm_with_hessian.h"
#include "g2o_core_api.h"

#include <memory>

namespace g2o {

  class BlockSolverBase;

  /**
   * \brief Implementation of Powell's Dogleg Algorithm
   */
  class G2O_CORE_API OptimizationAlgorithmDogleg : public OptimizationAlgorithmWithHessian
  {
    public:
      /** \brief type of the step to take */
      enum {
        STEP_UNDEFINED,
        STEP_SD, STEP_GN, STEP_DL
      };

    public:
      /**
       * construct the Dogleg algorithm, which will use the given Solver for solving the
       * linearized system.
       */
      explicit OptimizationAlgorithmDogleg(std::unique_ptr<BlockSolverBase> solver);
      virtual ~OptimizationAlgorithmDogleg();

      virtual SolverResult solve(int iteration, bool online = false);

      virtual void printVerbose(std::ostream& os) const;

      //! return the type of the last step taken by the algorithm
      int lastStep() const { return _lastStep;}
      //! return the diameter of the trust region
      number_t trustRegion() const { return _delta;}

      //! convert the type into an integer
      static const char* stepType2Str(int stepType);

    protected:
      // parameters
      Property<int>* _maxTrialsAfterFailure;
      Property<number_t>* _userDeltaInit;
      // damping to enforce positive definite matrix
      Property<number_t>* _initialLambda;
      Property<number_t>* _lamdbaFactor;

      VectorX _hsd;         ///< steepest decent step
      VectorX _hdl;         ///< final dogleg step
      VectorX _auxVector;   ///< auxilary vector used to perform multiplications or other stuff

      number_t _currentLambda;        ///< the damping factor to force positive definite matrix
      number_t _delta;                ///< trust region
      int _lastStep;                ///< type of the step taken by the algorithm
      bool _wasPDInAllIterations;   ///< the matrix we solve was positive definite in all iterations -> if not apply damping
      int _lastNumTries;

  private:
      std::unique_ptr<BlockSolverBase> m_solver;
  };

} // end namespace

#endif
