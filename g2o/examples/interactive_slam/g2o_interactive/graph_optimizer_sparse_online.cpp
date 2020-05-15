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

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

#include "graph_optimizer_sparse_online.h"

#include "g2o/stuff/macros.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace Eigen;

namespace g2o {

  namespace
  {
    template<int p, int l>
    std::unique_ptr<g2o::Solver> AllocatePCGSolver()
    {
      std::cerr << "# Using PCG online poseDim " << p << " landMarkDim " << l << " blockordering 1" << std::endl;

      auto linearSolver = g2o::make_unique<LinearSolverPCG<typename BlockSolverPL<p, l>::PoseMatrixType>>();
      linearSolver->setMaxIterations(6);
      return g2o::make_unique<BlockSolverPL<p, l>>(std::move(linearSolver));
    }
  }

  // force linking to the cholmod solver
  G2O_USE_OPTIMIZATION_LIBRARY(cholmod);

SparseOptimizerOnline::SparseOptimizerOnline(bool pcg) :
  SparseOptimizer(),
  slamDimension(3), newEdges(0), batchStep(true), vizWithGnuplot(false),
  _gnuplot(0), _usePcg(pcg), _underlyingSolver(0)
{
}

SparseOptimizerOnline::~SparseOptimizerOnline()
{
  if (_gnuplot) {
#ifdef WINDOWS
    _pclose(_gnuplot);
#else
    pclose(_gnuplot);
#endif
  }
}

int SparseOptimizerOnline::optimize(int iterations, bool online)
{
  //return SparseOptimizer::optimize(iterations, online);

  (void) iterations; // we only do one iteration anyhow
  OptimizationAlgorithm* solver = _algorithm;

  bool ok=true;

  solver->init(online);
  if (!online) {
    ok = _underlyingSolver->buildStructure();
    if (! ok) {
      cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
      return 0;
    }
  }

  if (_usePcg)
    batchStep = true;

  if (! online || batchStep) {
    //cerr << "BATCH" << endl;
    //_underlyingSolver->buildStructure();
    // copy over the updated estimate as new linearization point
    if (slamDimension == 3) {
      for (size_t i = 0; i < indexMapping().size(); ++i) {
        OnlineVertexSE2* v = static_cast<OnlineVertexSE2*>(indexMapping()[i]);
        v->setEstimate(v->updatedEstimate);
      }
    }
    else if (slamDimension == 6) {
      for (size_t i=0; i < indexMapping().size(); ++i) {
        OnlineVertexSE3* v = static_cast<OnlineVertexSE3*>(indexMapping()[i]);
        v->setEstimate(v->updatedEstimate);
      }
    }

    SparseOptimizer::computeActiveErrors();
    //SparseOptimizer::linearizeSystem();
    _underlyingSolver->buildSystem();
  }
  else {
    //cerr << "UPDATE" << endl;
    // compute the active errors for the required edges
    for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
      OptimizableGraph::Edge * e = static_cast<OptimizableGraph::Edge*>(*it);
      e->computeError();
    }
    // linearize the constraints and update the Hessian
    for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->linearizeOplus(jacobianWorkspace());
      e->constructQuadraticForm();
    }
    // update the b vector
    for (int i = 0; i < static_cast<int>(indexMapping().size()); ++i) {
      OptimizableGraph::Vertex* v = indexMapping()[i];
      int iBase = v->colInHessian();
      v->copyB(_underlyingSolver->b() + iBase);
    }
  }
  ok = _underlyingSolver->solve();
  update(_underlyingSolver->x());

  if (verbose()){
    computeActiveErrors();
    cerr
      << "nodes = " << vertices().size()
      << "\t edges= " << _activeEdges.size()
      << "\t chi2= " << FIXED(activeChi2())
      << endl;
  }

  if (vizWithGnuplot)
    gnuplotVisualization();

  if (! ok)
    return 0;
  return 1;
}

void SparseOptimizerOnline::update(double* update)
{
  if (slamDimension == 3) {
    for (size_t i=0; i < _ivMap.size(); ++i) {
      OnlineVertexSE2* v= static_cast<OnlineVertexSE2*>(_ivMap[i]);
      v->oplusUpdatedEstimate(update);
      update += 3;
    }
  }
  else if (slamDimension == 6) {
    for (size_t i=0; i < _ivMap.size(); ++i) {
      OnlineVertexSE3* v= static_cast<OnlineVertexSE3*>(_ivMap[i]);
      v->oplusUpdatedEstimate(update);
      update += 6;
    }
  }
}

bool SparseOptimizerOnline::updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset)
{
  newEdges = &eset;
  bool result = SparseOptimizer::updateInitialization(vset, eset);
  for (HyperGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->clearQuadraticForm(); // be sure that b is zero for this vertex
  }
  return result;
}

bool SparseOptimizerOnline::initSolver(int dimension, int /*batchEveryN*/)
{
  slamDimension = dimension;
  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  if (_usePcg) {

    std::unique_ptr<Solver> s;
    if (dimension == 3)
    {
      s = AllocatePCGSolver<3, 2>();
    }
    else
    {
      s = AllocatePCGSolver<6, 3>();
    }

    OptimizationAlgorithmGaussNewton* gaussNewton = new OptimizationAlgorithmGaussNewton(std::move(s));
    setAlgorithm(gaussNewton);
  }
  else {
    if (dimension == 3) {
      setAlgorithm(solverFactory->construct("gn_fix3_2_cholmod", solverProperty));
    } else {
      setAlgorithm(solverFactory->construct("gn_fix6_3_cholmod", solverProperty));
    }
  }

  OptimizationAlgorithmGaussNewton* gaussNewton = dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver());
  _underlyingSolver = &gaussNewton->solver();

  if (! solver()) {
    cerr << "Error allocating solver. Allocating CHOLMOD solver failed!" << endl;
    return false;
  }
  return true;
}

void SparseOptimizerOnline::gnuplotVisualization()
{
  if (slamDimension == 3) {
    if (! _gnuplot) {
#ifdef WINDOWS
      _gnuplot = _popen("gnuplot -persistent", "w");
#else
      _gnuplot = popen("gnuplot -persistent", "w");
#endif
      if (_gnuplot == 0)
        return;
      fprintf(_gnuplot, "set terminal X11 noraise\n");
      fprintf(_gnuplot, "set size ratio -1\n");
    }
    fprintf(_gnuplot, "plot \"-\" w l\n");
    for (EdgeSet::iterator it = edges().begin(); it != edges().end(); ++it) {
      OnlineEdgeSE2* e = static_cast<OnlineEdgeSE2*>(*it);
      OnlineVertexSE2* v1 = static_cast<OnlineVertexSE2*>(e->vertices()[0]);
      OnlineVertexSE2* v2 = static_cast<OnlineVertexSE2*>(e->vertices()[1]);
      fprintf(_gnuplot, "%f %f\n", v1->updatedEstimate.translation().x(), v1->updatedEstimate.translation().y());
      fprintf(_gnuplot, "%f %f\n\n", v2->updatedEstimate.translation().x(), v2->updatedEstimate.translation().y());
    }
    fprintf(_gnuplot, "e\n");
  }
  if (slamDimension == 6) {
    if (! _gnuplot) {
#ifdef WINDOWS
      _gnuplot = _popen("gnuplot -persistent", "w");
#else
      _gnuplot = popen("gnuplot -persistent", "w");
#endif
      if (_gnuplot == 0)
        return;
      fprintf(_gnuplot, "set terminal X11 noraise\n");
    }
    fprintf(_gnuplot, "splot \"-\" w l\n");
    for (EdgeSet::iterator it = edges().begin(); it != edges().end(); ++it) {
      OnlineEdgeSE3* e = (OnlineEdgeSE3*) *it;
      OnlineVertexSE3* v1 = static_cast<OnlineVertexSE3*>(e->vertices()[0]);
      OnlineVertexSE3* v2 = static_cast<OnlineVertexSE3*>(e->vertices()[1]);
      fprintf(_gnuplot, "%f %f %f\n", v1->updatedEstimate.translation().x(), v1->updatedEstimate.translation().y(), v1->updatedEstimate.translation().z());
      fprintf(_gnuplot, "%f %f %f \n\n\n", v2->updatedEstimate.translation().x(), v2->updatedEstimate.translation().y(), v2->updatedEstimate.translation().z());
    }
    fprintf(_gnuplot, "e\n");
  }
}

} // end namespace
