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

#ifndef G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H
#define G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H

#include <memory>

#include "g2o_core_api.h"
#include "optimization_algorithm_with_hessian.h"

namespace g2o {

/**
 * \brief Implementation of the Gauss Newton Algorithm
 */
class G2O_CORE_API OptimizationAlgorithmGaussNewton
    : public OptimizationAlgorithmWithHessian {
 public:
  /**
   * construct the Gauss Newton algorithm, which use the given Solver for
   * solving the linearized system.
   */
  explicit OptimizationAlgorithmGaussNewton(std::unique_ptr<Solver> solver);
  virtual ~OptimizationAlgorithmGaussNewton();

  virtual SolverResult solve(int iteration, bool online = false);

  virtual void printVerbose(std::ostream& os) const;

 private:
  std::unique_ptr<Solver> m_solver;
};

}  // namespace g2o

#endif
