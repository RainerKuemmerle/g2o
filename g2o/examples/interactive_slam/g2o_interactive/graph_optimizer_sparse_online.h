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

#ifndef G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H
#define G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H

#include "g2o_interactive_api.h"
#include "g2o/core/sparse_optimizer.h"

namespace g2o {

  class Solver;

  class G2O_INTERACTIVE_API SparseOptimizerOnline : public SparseOptimizer
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
