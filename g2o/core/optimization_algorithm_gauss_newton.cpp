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

#include "optimization_algorithm_gauss_newton.h"

#include <iostream>

#include "g2o/stuff/timeutil.h"
#include "g2o/stuff/macros.h"

#include "solver.h"
#include "batch_stats.h"
#include "sparse_optimizer.h"
using namespace std;

namespace g2o {

  OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(Solver* solver) :
    OptimizationAlgorithmWithHessian(solver)
  {
  }

  OptimizationAlgorithmGaussNewton::~OptimizationAlgorithmGaussNewton()
  {
  }

  OptimizationAlgorithm::SolverResult OptimizationAlgorithmGaussNewton::solve(int iteration, bool online)
  {
    assert(_optimizer && "_optimizer not set");
    assert(_solver->optimizer() == _optimizer && "underlying linear solver operates on different graph");
    bool ok = true;
    if (iteration == 0 && !online) { // built up the CCS structure, here due to easy time measure
      ok = _solver->buildStructure();
      if (! ok) {
        cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
        return OptimizationAlgorithm::Fail;
      }
    }

    double t=get_time();
    _optimizer->computeActiveErrors();
    if (globalStats) {
      globalStats->timeResiduals = get_time()-t;
      t=get_time();
    }

    _optimizer->linearizeSystem();
    if (globalStats) {
      globalStats->timeLinearize = get_time()-t;
      t=get_time();
    }

    _solver->buildSystem();
    if (globalStats) {
      globalStats->timeQuadraticForm = get_time()-t;
    }

    t=get_time();
    ok = _solver->solve();
    if (globalStats)
      globalStats->timeLinearSolution = get_time()-t;

    t=get_time();
    _optimizer->update(_solver->x());
    if (globalStats) {
      globalStats->timeUpdate = get_time()-t;
    }
    if (ok)
      return OK;
    else
      return Fail;
  }

  void OptimizationAlgorithmGaussNewton::printVerbose(std::ostream& os) const
  {
    os
      << "\t schur= " << _solver->schur();
  }

} // end namespace
