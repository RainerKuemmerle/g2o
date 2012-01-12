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

#include "optimization_algorithm_with_hessian.h"

#include "solver.h"
#include "optimizable_graph.h"
#include "sparse_optimizer.h"

#include <iostream>
using namespace std;

namespace g2o {

  OptimizationAlgorithmWithHessian::OptimizationAlgorithmWithHessian(Solver* solver) :
    OptimizationAlgorithm(),
    _solver(solver)
  {
  }

  OptimizationAlgorithmWithHessian::~OptimizationAlgorithmWithHessian()
  {
    delete _solver;
  }

  bool OptimizationAlgorithmWithHessian::init(bool online)
  {
    assert(_optimizer && "_optimizer not set");
    assert(_solver && "Solver not set");
    bool useSchur=false;
    for (OptimizableGraph::VertexContainer::const_iterator it=_optimizer->activeVertices().begin(); it!=_optimizer->activeVertices().end(); ++it) {
      OptimizableGraph::Vertex* v= *it;
      if (v->marginalized()){
        useSchur=true;
        break;
      }
    }
    if (useSchur){
      if  (_solver->supportsSchur())
        _solver->setSchur(true);
    } else {
      if  (_solver->supportsSchur())
        _solver->setSchur(false);
    }

    bool initState = _solver->init(_optimizer, online);
    return initState;
  }

  bool OptimizationAlgorithmWithHessian::computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices)
  {
    return _solver ? _solver->computeMarginals(spinv, blockIndices) : false;
  }

  bool OptimizationAlgorithmWithHessian::buildLinearStructure()
  {
    return _solver ? _solver->buildStructure() : false;
  }

  void OptimizationAlgorithmWithHessian::updateLinearSystem()
  {
    if (_solver)
      _solver->buildSystem();
  }

  bool OptimizationAlgorithmWithHessian::updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges)
  {
    return _solver ? _solver->updateStructure(vset, edges) : false;
  }

} // end namespace
