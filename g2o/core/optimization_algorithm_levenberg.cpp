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

#include "optimization_algorithm_levenberg.h"

#include <iostream>

#include "g2o/stuff/timeutil.h"

#include "sparse_optimizer.h"
#include "solver.h"
#include "batch_stats.h"
using namespace std;

namespace g2o {

  OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg(Solver* solver) :
    OptimizationAlgorithmWithHessian(solver)
  {
    _currentLambda = -1.;
    _tau = 1e-5;
    _goodStepUpperScale = 2./3.;
    _goodStepLowerScale = 1./3.;
    _userLambdaInit = _properties.makeProperty<Property<double> >("initialLambda", 0.);
    _maxTrialsAfterFailure = _properties.makeProperty<Property<int> >("maxTrialsAfterFailure", 10);
    _ni=2.;
  }

  OptimizationAlgorithmLevenberg::~OptimizationAlgorithmLevenberg()
  {
  }

  OptimizationAlgorithm::SolverResult OptimizationAlgorithmLevenberg::solve(int iteration, bool online)
  {
    assert(_optimizer && "_optimizer not set");
    assert(_solver->optimizer() == _optimizer && "underlying linear solver operates on different graph");

    if (iteration == 0 && !online) { // built up the CCS structure, here due to easy time measure
      bool ok = _solver->buildStructure();
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

    double currentChi = _optimizer->activeChi2();
    double tempChi=currentChi;

    _optimizer->linearizeSystem();
    if (globalStats) {
      globalStats->timeLinearize = get_time()-t;
      t=get_time();
    }

    _solver->buildSystem();
    if (globalStats) {
      globalStats->timeQuadraticForm = get_time()-t;
    }

    // core part of the Levenbarg algorithm
    if (iteration == 0) {
      _currentLambda = computeLambdaInit();
    }

    double rho=0;
    int qmax=0;
    do {
      _optimizer->push();
      if (globalStats) {
        globalStats->levenbergIterations++;
        t=get_time();
      }
      // update the diagonal of the system matrix
      _solver->setLambda(_currentLambda);
      bool ok2 = _solver->solve();
      if (globalStats) {
        globalStats->timeLinearSolution+=get_time()-t;
        t=get_time();
      }
      _optimizer->update(_solver->x());
      if (globalStats) {
        globalStats->timeUpdate = get_time()-t;
      }

      // restore the diagonal
      _solver->setLambda(- _currentLambda);

      _optimizer->computeActiveErrors();
      tempChi = _optimizer->activeChi2();

      if (! ok2)
        tempChi=std::numeric_limits<double>::max();

      rho = (currentChi-tempChi);
      double scale = computeScale();
      scale += 1e-3; // make sure it's non-zero :)
      rho /=  scale;

      if (rho>0){ // last step was good
        double alpha = 1.-pow((2*rho-1),3);
        // crop lambda between minimum and maximum factors
        alpha = (std::min)(alpha, _goodStepUpperScale);
        double scaleFactor = (std::max)(_goodStepLowerScale, alpha);
        _currentLambda *= scaleFactor;
        _ni = 2;
        currentChi=tempChi;
        _optimizer->discardTop();
      } else {
        _currentLambda*=_ni;
        _ni*=2;
        _optimizer->pop(); // restore the last state before trying to optimize
      }
      qmax++;
    } while (rho<0 && qmax < _maxTrialsAfterFailure->value() && ! _optimizer->terminate());

    if (qmax == _maxTrialsAfterFailure->value() || rho==0)
      return Terminate;
    return OK;
  }

  double OptimizationAlgorithmLevenberg::computeLambdaInit() const
  {
    if (_userLambdaInit->value() > 0)
      return _userLambdaInit->value();
    double maxDiagonal=0.;
    for (size_t k = 0; k < _optimizer->indexMapping().size(); k++) {
      OptimizableGraph::Vertex* v = _optimizer->indexMapping()[k];
      assert(v);
      int dim = v->dimension();
      for (int j = 0; j < dim; ++j){
        maxDiagonal = std::max(fabs(v->hessian(j,j)),maxDiagonal); 
      }
    }
    return _tau*maxDiagonal;
  }

  double OptimizationAlgorithmLevenberg::computeScale() const
  {
    double scale = 0.;
    for (size_t j=0; j < _solver->vectorSize(); j++){
      scale += _solver->x()[j] * (_currentLambda * _solver->x()[j] + _solver->b()[j]);
    }
    return scale;
  }

  void OptimizationAlgorithmLevenberg::setMaxTrialsAfterFailure(int max_trials)
  {
    _maxTrialsAfterFailure->setValue(max_trials);
  }

  void OptimizationAlgorithmLevenberg::setUserLambdaInit(double lambda)
  {
    _userLambdaInit->setValue(lambda);
  }

  void OptimizationAlgorithmLevenberg::printVerbose(std::ostream& os) const
  {
    os
      << "\t schur= " << _solver->schur()
      << "\t lambda= " << FIXED(_currentLambda);
  }

} // end namespace
