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

#ifndef G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H
#define G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H

#include "g2o/core/sparse_optimizer.h"

namespace g2o {

  class Solver;

  class SparseOptimizerOnline : public SparseOptimizer
  {
    public:
      explicit SparseOptimizerOnline(bool pcg=false);
      virtual ~SparseOptimizerOnline();

      int optimize(int iterations, bool online = false);

      virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);

      void update(double* update);

      virtual bool initSolver(int dimension, int batchEveryN);

    public:
      int slamDimension;

      HyperGraph::EdgeSet* newEdges;

      bool batchStep;
      bool vizWithGnuplot;

      virtual void gnuplotVisualization();
      
    protected:
      FILE* _gnuplot;
      bool _usePcg;
      Solver* _underlyingSolver;
  };

} // end namespace

#endif
