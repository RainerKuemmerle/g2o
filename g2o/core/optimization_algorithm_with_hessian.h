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

#ifndef G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H
#define G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H

#include "optimization_algorithm.h"

namespace g2o {

  class Solver;

  /**
   * \brief Base for solvers operating on the approximated Hessian, e.g., Gauss-Newton, Levenberg
   */
  class OptimizationAlgorithmWithHessian : public OptimizationAlgorithm
  {
    public:
      explicit OptimizationAlgorithmWithHessian(Solver* solver);
      virtual ~OptimizationAlgorithmWithHessian();     

      virtual bool init(bool online = false);

      virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);

      virtual bool buildLinearStructure();

      virtual void updateLinearSystem();

      virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges);

      //! return the underlying solver used to solve the linear system
      Solver* solver() { return _solver;}

    protected:
      Solver* _solver;
  };

}// end namespace

#endif
