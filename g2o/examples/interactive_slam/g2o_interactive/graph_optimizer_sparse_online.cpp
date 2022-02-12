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

#include "graph_optimizer_sparse_online.h"

#include <fstream>
#include <iomanip>
#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/stuff/macros.h"
#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

namespace g2o {

namespace {
template <int P, int L>
std::unique_ptr<g2o::Solver> AllocatePCGSolver() {
  std::cerr << "# Using PCG online poseDim " << P << " landMarkDim " << L
            << " blockordering 1" << std::endl;

  auto linearSolver = g2o::make_unique<
      LinearSolverPCG<typename BlockSolverPL<P, L>::PoseMatrixType>>();
  linearSolver->setMaxIterations(6);
  return g2o::make_unique<BlockSolverPL<P, L>>(std::move(linearSolver));
}
}  // namespace

// force linking to the cholmod solver
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);

SparseOptimizerOnline::SparseOptimizerOnline(bool pcg) : usePcg_(pcg) {}

SparseOptimizerOnline::~SparseOptimizerOnline() {
  if (gnuplot_) {
#ifdef WINDOWS
    _pclose(_gnuplot);
#else
    pclose(gnuplot_);
#endif
  }
}

int SparseOptimizerOnline::optimize(int iterations, bool online) {
  // return SparseOptimizer::optimize(iterations, online);

  (void)iterations;  // we only do one iteration anyhow

  bool ok = true;

  algorithm_->init(online);
  if (!online) {
    ok = underlyingSolver_->buildStructure();
    if (!ok) {
      std::cerr << __PRETTY_FUNCTION__
                << ": Failure while building CCS structure" << std::endl;
      return 0;
    }
  }

  if (usePcg_) batchStep = true;

  if (!online || batchStep) {
    // cerr << "BATCH" << endl;
    //_underlyingSolver->buildStructure();
    //  copy over the updated estimate as new linearization point
    if (slamDimension == 3) {
      for (auto* i : indexMapping()) {
        auto* v = static_cast<OnlineVertexSE2*>(i);
        v->setEstimate(v->updatedEstimate);
      }
    } else if (slamDimension == 6) {
      for (auto* i : indexMapping()) {
        auto* v = static_cast<OnlineVertexSE3*>(i);
        v->setEstimate(v->updatedEstimate);
      }
    }

    SparseOptimizer::computeActiveErrors();
    // SparseOptimizer::linearizeSystem();
    underlyingSolver_->buildSystem();
  } else {
    // cerr << "UPDATE" << endl;
    //  compute the active errors for the required edges
    for (const auto& newEdge : *newEdges) {
      auto* e = static_cast<OptimizableGraph::Edge*>(newEdge.get());
      e->computeError();
    }
    // linearize the constraints and update the Hessian
    for (const auto& newEdge : *newEdges) {
      auto* e = static_cast<OptimizableGraph::Edge*>(newEdge.get());
      e->linearizeOplus(jacobianWorkspace());
      e->constructQuadraticForm();
    }
    // update the b vector
    for (auto* v : indexMapping()) {
      int iBase = v->colInHessian();
      v->copyB(underlyingSolver_->b() + iBase);
    }
  }
  ok = underlyingSolver_->solve();
  update(underlyingSolver_->x());

  if (verbose()) {
    computeActiveErrors();
    std::cerr << "nodes = " << vertices().size()
              << "\t edges= " << activeEdges_.size()
              << "\t chi2= " << FIXED(activeChi2()) << std::endl;
  }

  if (vizWithGnuplot) gnuplotVisualization();

  if (!ok) return 0;
  return 1;
}

void SparseOptimizerOnline::update(double* update) {
  if (slamDimension == 3) {
    for (auto& i : ivMap_) {
      auto* v = static_cast<OnlineVertexSE2*>(i);
      v->oplusUpdatedEstimate(update);
      update += 3;
    }
  } else if (slamDimension == 6) {
    for (auto& i : ivMap_) {
      auto* v = static_cast<OnlineVertexSE3*>(i);
      v->oplusUpdatedEstimate(update);
      update += 6;
    }
  }
}

bool SparseOptimizerOnline::updateInitialization(HyperGraph::VertexSet& vset,
                                                 HyperGraph::EdgeSet& eset) {
  newEdges = &eset;
  bool result = SparseOptimizer::updateInitialization(vset, eset);
  for (const auto& it : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    v->clearQuadraticForm();  // be sure that b is zero for this vertex
  }
  return result;
}

bool SparseOptimizerOnline::initSolver(int dimension, int /*batchEveryN*/) {
  slamDimension = dimension;
  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  if (usePcg_) {
    std::unique_ptr<Solver> s =
        dimension == 3 ? AllocatePCGSolver<3, 2>() : AllocatePCGSolver<6, 3>();

    auto* gaussNewton = new OptimizationAlgorithmGaussNewton(std::move(s));
    setAlgorithm(std::unique_ptr<OptimizationAlgorithm>(gaussNewton));
  } else {
    if (dimension == 3) {
      setAlgorithm(
          solverFactory->construct("gn_fix3_2_cholmod", solverProperty));
    } else {
      setAlgorithm(
          solverFactory->construct("gn_fix6_3_cholmod", solverProperty));
    }
  }

  OptimizationAlgorithmGaussNewton* gaussNewton =
      dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver().get());
  underlyingSolver_ = &gaussNewton->solver();

  if (!solver()) {
    std::cerr << "Error allocating solver. Allocating CHOLMOD solver failed!"
              << std::endl;
    return false;
  }
  return true;
}

void SparseOptimizerOnline::gnuplotVisualization() {
  if (slamDimension == 3) {
    if (!gnuplot_) {
#ifdef WINDOWS
      _gnuplot = _popen("gnuplot -persistent", "w");
#else
      gnuplot_ = popen("gnuplot -persistent", "w");
#endif
      if (gnuplot_ == nullptr) return;
      fprintf(gnuplot_, "set terminal X11 noraise\n");
      fprintf(gnuplot_, "set size ratio -1\n");
    }
    fprintf(gnuplot_, "plot \"-\" w l\n");
    for (const auto& it : edges()) {
      auto* e = static_cast<OnlineEdgeSE2*>(it.get());
      auto* v1 = static_cast<OnlineVertexSE2*>(e->vertices()[0].get());
      auto* v2 = static_cast<OnlineVertexSE2*>(e->vertices()[1].get());
      fprintf(gnuplot_, "%f %f\n", v1->updatedEstimate.translation().x(),
              v1->updatedEstimate.translation().y());
      fprintf(gnuplot_, "%f %f\n\n", v2->updatedEstimate.translation().x(),
              v2->updatedEstimate.translation().y());
    }
    fprintf(gnuplot_, "e\n");
  }
  if (slamDimension == 6) {
    if (!gnuplot_) {
#ifdef WINDOWS
      _gnuplot = _popen("gnuplot -persistent", "w");
#else
      gnuplot_ = popen("gnuplot -persistent", "w");
#endif
      if (gnuplot_ == nullptr) return;
      fprintf(gnuplot_, "set terminal X11 noraise\n");
    }
    fprintf(gnuplot_, "splot \"-\" w l\n");
    for (const auto& it : edges()) {
      auto* e = static_cast<OnlineEdgeSE3*>(it.get());
      auto* v1 = static_cast<OnlineVertexSE3*>(e->vertices()[0].get());
      auto* v2 = static_cast<OnlineVertexSE3*>(e->vertices()[1].get());
      fprintf(gnuplot_, "%f %f %f\n", v1->updatedEstimate.translation().x(),
              v1->updatedEstimate.translation().y(),
              v1->updatedEstimate.translation().z());
      fprintf(gnuplot_, "%f %f %f \n\n\n",
              v2->updatedEstimate.translation().x(),
              v2->updatedEstimate.translation().y(),
              v2->updatedEstimate.translation().z());
    }
    fprintf(gnuplot_, "e\n");
  }
}

}  // namespace g2o
