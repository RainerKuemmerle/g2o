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

#include "sparse_optimizer.h"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <iterator>

#include "batch_stats.h"
#include "estimate_propagator.h"
#include "g2o/config.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/timeutil.h"
#include "hyper_graph_action.h"
#include "optimization_algorithm.h"
#include "robust_kernel.h"

namespace g2o {

SparseOptimizer::SparseOptimizer() : algorithm_(nullptr) {
  graphActions_.resize(kAtNumElements);
}

SparseOptimizer::~SparseOptimizer() {
  G2OBatchStatistics::setGlobalStats(nullptr);
}

void SparseOptimizer::computeActiveErrors() {
  // call the callbacks in case there is something registered
  HyperGraphActionSet& actions = graphActions_[kAtComputeactiverror];
  if (!actions.empty()) {
    for (const auto& action : actions) (*action)(*this);
  }

#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (activeEdges_.size() > 50)
#endif
  for (auto& _activeEdge : activeEdges_) {
    OptimizableGraph::Edge* e = _activeEdge.get();
    e->computeError();
  }

#ifndef NDEBUG
  for (auto & activeEdge : activeEdges_) {
    OptimizableGraph::Edge* e = activeEdge.get();
    bool hasNan = arrayHasNaN(e->errorData(), e->dimension());
    if (hasNan) {
      std::cerr << "computeActiveErrors(): found NaN in error for edge " << e
                << std::endl;
    }
  }
#endif
}

number_t SparseOptimizer::activeChi2() const {
  number_t chi = 0.0;
  for (const auto& _activeEdge : activeEdges_) {
    const OptimizableGraph::Edge* e = _activeEdge.get();
    chi += e->chi2();
  }
  return chi;
}

number_t SparseOptimizer::activeRobustChi2() const {
  Vector3 rho;
  number_t chi = 0.0;
  for (const auto& _activeEdge : activeEdges_) {
    const OptimizableGraph::Edge* e = _activeEdge.get();
    if (e->robustKernel()) {
      e->robustKernel()->robustify(e->chi2(), rho);
      chi += rho[0];
    } else
      chi += e->chi2();
  }
  return chi;
}

std::shared_ptr<OptimizableGraph::Vertex> SparseOptimizer::findGauge() {
  if (vertices().empty()) return nullptr;

  const int maxDim = maxDimension();

  std::shared_ptr<OptimizableGraph::Vertex> rut;
  for (auto& it : vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    if (v->dimension() == maxDim) {
      rut = std::static_pointer_cast<OptimizableGraph::Vertex>(it.second);
      break;
    }
  }
  return rut;
}

bool SparseOptimizer::gaugeFreedom() {
  if (vertices().empty()) return false;

  const int maxDim = maxDimension();

  for (auto& it : vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    if (v->dimension() == maxDim) {
      // test for fixed vertex
      if (v->fixed()) {
        return false;
      }
      // test for full dimension prior
      for (const auto& eit : v->edges()) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(eit.lock());
        if (e->vertices().size() == 1 && e->dimension() == maxDim) return false;
      }
    }
  }
  return true;
}

bool SparseOptimizer::buildIndexMapping(
    SparseOptimizer::VertexContainer& vlist) {
  if (vlist.empty()) {
    ivMap_.clear();
    return false;
  }

  ivMap_.resize(vlist.size());
  size_t i = 0;
  for (int k = 0; k < 2; k++)
    for (auto& it : vlist) {
      OptimizableGraph::Vertex* v = it.get();
      if (!v->fixed()) {
        if (static_cast<int>(v->marginalized()) == k) {
          v->setHessianIndex(i);
          ivMap_[i] = v;
          i++;
        }
      } else {
        v->setHessianIndex(-1);
      }
    }
  ivMap_.resize(i);
  return true;
}

void SparseOptimizer::clearIndexMapping() {
  for (auto& i : ivMap_) {
    i->setHessianIndex(-1);
    i = nullptr;
  }
}

bool SparseOptimizer::initializeOptimization(int level) {
  HyperGraph::VertexSet vset;
  for (auto& it : vertices()) vset.insert(it.second);
  return initializeOptimization(vset, level);
}

bool SparseOptimizer::initializeOptimization(HyperGraph::VertexSet& vset,
                                             int level) {
  if (edges().empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Attempt to initialize an empty graph"
              << std::endl;
    return false;
  }
  preIteration(-1);
  bool workspaceAllocated = jacobianWorkspace_.allocate();
  (void)workspaceAllocated;
  assert(workspaceAllocated &&
         "Error while allocating memory for the Jacobians");
  clearIndexMapping();
  activeVertices_.clear();
  activeVertices_.reserve(vset.size());
  activeEdges_.clear();
  EdgeSet auxEdgeSet;  // temporary structure to avoid duplicates
  for (auto it = vset.begin(); it != vset.end(); ++it) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it->get());
    const OptimizableGraph::EdgeSetWeak& vEdges = v->edges();
    // count if there are edges in that level. If not remove from the pool
    int levelEdges = 0;
    for (const auto& vEdge : vEdges) {
      auto e = std::static_pointer_cast<OptimizableGraph::Edge>(vEdge.lock());
      if (level < 0 || e->level() == level) {
        bool allVerticesOK = true;
        for (const auto& vit : e->vertices()) {
          if (vset.find(vit) == vset.end()) {
            allVerticesOK = false;
            break;
          }
        }
        if (allVerticesOK && !e->allVerticesFixed()) {
          auxEdgeSet.insert(e);
          levelEdges++;
        }
      }
    }
    if (levelEdges) {
      activeVertices_.push_back(std::static_pointer_cast<Vertex>(*it));

      // test for NANs in the current estimate if we are debugging
#ifndef NDEBUG
      int estimateDim = v->estimateDimension();
      if (estimateDim > 0) {
        VectorX estimateData(estimateDim);
        if (v->getEstimateData(estimateData.data())) {
          int k;
          bool hasNan = arrayHasNaN(estimateData.data(), estimateDim, &k);
          if (hasNan)
            std::cerr << __PRETTY_FUNCTION__ << ": Vertex " << v->id()
                      << " contains a nan entry at index " << k << std::endl;
        }
      }
#endif
    }
  }

  activeEdges_.reserve(auxEdgeSet.size());
  for (const auto& it : auxEdgeSet)
    activeEdges_.push_back(std::static_pointer_cast<Edge>(it));

  sortVectorContainers();
  bool indexMappingStatus = buildIndexMapping(activeVertices_);
  postIteration(-1);
  return indexMappingStatus;
}

bool SparseOptimizer::initializeOptimization(HyperGraph::EdgeSet& eset) {
  preIteration(-1);
  bool workspaceAllocated = jacobianWorkspace_.allocate();
  (void)workspaceAllocated;
  assert(workspaceAllocated &&
         "Error while allocating memory for the Jacobians");
  clearIndexMapping();
  activeVertices_.clear();
  activeEdges_.clear();
  activeEdges_.reserve(eset.size());
  VertexSet auxVertexSet;  // temporary structure to avoid duplicates
  for (const auto& it : eset) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    if (e->numUndefinedVertices()) continue;
    for (auto& vit : e->vertices()) {
      auxVertexSet.insert(vit);
    }
    activeEdges_.push_back(std::static_pointer_cast<Edge>(it));
  }

  activeVertices_.reserve(auxVertexSet.size());
  for (const auto& it : auxVertexSet)
    activeVertices_.push_back(std::static_pointer_cast<Vertex>(it));

  sortVectorContainers();
  bool indexMappingStatus = buildIndexMapping(activeVertices_);
  postIteration(-1);
  return indexMappingStatus;
}

void SparseOptimizer::setToOrigin() {
  for (auto& it : vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    v->setToOrigin();
  }
}

void SparseOptimizer::computeInitialGuess() {
  EstimatePropagator::PropagateCost costFunction(this);
  computeInitialGuess(costFunction);
}

void SparseOptimizer::computeInitialGuess(EstimatePropagatorCost& propagator) {
  OptimizableGraph::VertexSet emptySet;
  std::set<Vertex*> backupVertices;
  OptimizableGraph::VertexSet fixedVertices;  // these are the root nodes where
                                              // to start the initialization
  for (auto& e : activeEdges_) {
    for (size_t i = 0; i < e->vertices().size(); ++i) {
      auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertex(i));
      if (!v) continue;
      if (v->fixed())
        fixedVertices.insert(v);
      else {  // check for having a prior which is able to fully initialize a
              // vertex
        for (auto vedgeIt = v->edges().begin(); vedgeIt != v->edges().end();
             ++vedgeIt) {
          auto vedge =
              std::static_pointer_cast<OptimizableGraph::Edge>(vedgeIt->lock());
          if (vedge->vertices().size() == 1 &&
              vedge->initialEstimatePossible(emptySet, v.get()) > 0.) {
            // cerr << "Initialize with prior for " << v->id() << endl;
            vedge->initialEstimate(emptySet, v.get());
            fixedVertices.insert(v);
          }
        }
      }
      if (v->hessianIndex() == -1) {
        auto foundIt = backupVertices.find(v.get());
        if (foundIt == backupVertices.end()) {
          v->push();
          backupVertices.insert(v.get());
        }
      }
    }
  }

  EstimatePropagator estimatePropagator(this);
  estimatePropagator.propagate(fixedVertices, propagator);

  // restoring the vertices that should not be initialized
  for (auto* v : backupVertices) {
    v->pop();
  }
  if (verbose()) {
    computeActiveErrors();
    std::cerr << "iteration= -1\t chi2= " << activeChi2() << "\t time= 0.0"
              << "\t cumTime= 0.0"
              << "\t (using initial guess from " << propagator.name() << ")"
              << std::endl;
  }
}

int SparseOptimizer::optimize(int iterations, bool online) {
  if (ivMap_.empty()) {
    std::cerr << __PRETTY_FUNCTION__
              << ": 0 vertices to optimize, maybe forgot to call "
                 "initializeOptimization()"
              << std::endl;
    return -1;
  }

  int cjIterations = 0;
  number_t cumTime = 0;
  bool ok = true;

  ok = algorithm_->init(online);
  if (!ok) {
    std::cerr << __PRETTY_FUNCTION__ << " Error while initializing"
              << std::endl;
    return -1;
  }

  batchStatistics_.clear();
  if (computeBatchStatistics_) batchStatistics_.resize(iterations);

  OptimizationAlgorithm::SolverResult result = OptimizationAlgorithm::kOk;
  for (int i = 0; i < iterations && !terminate() && ok; i++) {
    preIteration(i);

    if (computeBatchStatistics_) {
      G2OBatchStatistics& cstat = batchStatistics_[i];
      G2OBatchStatistics::setGlobalStats(&cstat);
      cstat.iteration = i;
      cstat.numEdges = activeEdges_.size();
      cstat.numVertices = activeVertices_.size();
    }

    number_t ts = get_monotonic_time();
    result = algorithm_->solve(i, online);
    ok = (result == OptimizationAlgorithm::kOk);

    bool errorComputed = false;
    if (computeBatchStatistics_) {
      computeActiveErrors();
      errorComputed = true;
      batchStatistics_[i].chi2 = activeRobustChi2();
      batchStatistics_[i].timeIteration = get_monotonic_time() - ts;
    }

    if (verbose()) {
      number_t dts = get_monotonic_time() - ts;
      cumTime += dts;
      if (!errorComputed) computeActiveErrors();
      std::cerr << "iteration= " << i
                << "\t chi2= " << FIXED(activeRobustChi2())
                << "\t time= " << dts << "\t cumTime= " << cumTime
                << "\t edges= " << activeEdges_.size();
      algorithm_->printVerbose(std::cerr);
      std::cerr << std::endl;
    }
    ++cjIterations;
    postIteration(i);
  }
  if (result == OptimizationAlgorithm::kFail) {
    return 0;
  }
  return cjIterations;
}

void SparseOptimizer::update(number_t* update) {
  // update the graph by calling oplus on the vertices
  VectorX::MapType updateMap(nullptr, 42);
  for (auto* v : ivMap_) {
#ifndef NDEBUG
    bool hasNan = arrayHasNaN(update, v->dimension());
    if (hasNan)
      std::cerr << __PRETTY_FUNCTION__ << ": Update contains a nan for vertex "
                << v->id() << std::endl;
#endif
    new (&updateMap) VectorX::MapType(update, v->dimension());
    v->oplus(updateMap);
    update += v->dimension();
  }
}

void SparseOptimizer::setComputeBatchStatistics(bool computeBatchStatistics) {
  if (computeBatchStatistics_ && !computeBatchStatistics) {
    G2OBatchStatistics::setGlobalStats(nullptr);
    batchStatistics_.clear();
  }
  computeBatchStatistics_ = computeBatchStatistics;
}

bool SparseOptimizer::updateInitialization(HyperGraph::VertexSet& vset,
                                           HyperGraph::EdgeSet& eset) {
  HyperGraph::VertexContainer newVertices;
  newVertices.reserve(vset.size());
  activeVertices_.reserve(activeVertices_.size() + vset.size());
  activeEdges_.reserve(activeEdges_.size() + eset.size());
  for (const auto& it : eset) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it);
    if (!e->allVerticesFixed()) activeEdges_.push_back(e);
  }

  // update the index mapping
  size_t next = ivMap_.size();
  for (const auto& it : vset) {
    auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(it);
    if (!v->fixed()) {
      if (!v->marginalized()) {
        v->setHessianIndex(next);
        ivMap_.push_back(v.get());
        newVertices.push_back(v);
        activeVertices_.push_back(v);
        next++;
      } else  // not supported right now
        abort();
    } else {
      v->setHessianIndex(-1);
    }
  }

  // if (newVertices.size() != vset.size())
  // cerr << __PRETTY_FUNCTION__ << ": something went wrong " <<
  // PVAR(vset.size()) << " " << PVAR(newVertices.size()) << endl;
  return algorithm_->updateStructure(newVertices, eset);
}

void SparseOptimizer::sortVectorContainers() {
  // sort vector structures to get deterministic ordering based on IDs
  sort(activeVertices_.begin(), activeVertices_.end(), VertexIDCompare());
  sort(activeEdges_.begin(), activeEdges_.end(), EdgeIDCompare());
}

void SparseOptimizer::clear() {
  ivMap_.clear();
  activeVertices_.clear();
  activeEdges_.clear();
  OptimizableGraph::clear();
}

SparseOptimizer::VertexContainer::const_iterator
SparseOptimizer::findActiveVertex(const OptimizableGraph::Vertex* v) const {
  auto lower = lower_bound(activeVertices_.begin(), activeVertices_.end(), v,
                           VertexIDCompare());
  if (lower == activeVertices_.end()) return activeVertices_.end();
  if (lower->get() == v) return lower;
  return activeVertices_.end();
}

SparseOptimizer::EdgeContainer::const_iterator SparseOptimizer::findActiveEdge(
    const OptimizableGraph::Edge* e) const {
  auto lower =
      lower_bound(activeEdges_.begin(), activeEdges_.end(), e, EdgeIDCompare());
  if (lower == activeEdges_.end()) return activeEdges_.end();
  if (lower->get() == e) return lower;
  return activeEdges_.end();
}

void SparseOptimizer::push(SparseOptimizer::VertexContainer& vlist) {
  for (auto& it : vlist) it->push();
}

void SparseOptimizer::pop(SparseOptimizer::VertexContainer& vlist) {
  for (auto& it : vlist) it->pop();
}

void SparseOptimizer::push(HyperGraph::VertexSet& vlist) {
  OptimizableGraph::push(vlist);
}

void SparseOptimizer::pop(HyperGraph::VertexSet& vlist) {
  OptimizableGraph::pop(vlist);
}

void SparseOptimizer::discardTop(SparseOptimizer::VertexContainer& vlist) {
  for (auto& it : vlist) it->discardTop();
}

void SparseOptimizer::setVerbose(bool verbose) { verbose_ = verbose; }

void SparseOptimizer::setAlgorithm(
    const std::shared_ptr<OptimizationAlgorithm>& algorithm) {
  if (algorithm_)  // reset the optimizer for the formerly used solver
    algorithm_->setOptimizer(nullptr);

  algorithm_ = algorithm;

  if (algorithm_) algorithm_->setOptimizer(this);
}

bool SparseOptimizer::computeMarginals(
    SparseBlockMatrix<MatrixX>& spinv,
    const std::vector<std::pair<int, int> >& blockIndices) {
  return algorithm_->computeMarginals(spinv, blockIndices);
}

void SparseOptimizer::setForceStopFlag(bool* flag) { forceStopFlag_ = flag; }

bool SparseOptimizer::removeVertex(const std::shared_ptr<HyperGraph::Vertex>& v,
                                   bool detach) {
  auto* vv = static_cast<OptimizableGraph::Vertex*>(v.get());
  if (vv->hessianIndex() >= 0) {
    clearIndexMapping();
    ivMap_.clear();
  }
  return OptimizableGraph::removeVertex(v, detach);
}

bool SparseOptimizer::addComputeErrorAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      graphActions_[kAtComputeactiverror].insert(action);
  return insertResult.second;
}

bool SparseOptimizer::removeComputeErrorAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  return graphActions_[kAtComputeactiverror].erase(action) > 0;
}

void SparseOptimizer::push() { push(activeVertices_); }

void SparseOptimizer::pop() { pop(activeVertices_); }

void SparseOptimizer::discardTop() { discardTop(activeVertices_); }

}  // namespace g2o
