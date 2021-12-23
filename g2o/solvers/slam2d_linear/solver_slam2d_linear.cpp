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

#include "solver_slam2d_linear.h"

#include <Eigen/Core>

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/stuff/misc.h"

#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "g2o/core/solver.h"

using namespace std;

namespace g2o {

  /**
   * \brief compute the initial guess of theta while travelling along the MST
   */
  class ThetaTreeAction : public HyperDijkstra::TreeAction
  {
    public:
      explicit ThetaTreeAction(VectorX& theta) : HyperDijkstra::TreeAction(), thetaGuess_(theta) {}
      number_t perform(const std::shared_ptr<HyperGraph::Vertex>& v,
                               const std::shared_ptr<HyperGraph::Vertex>& vParent,
                               const std::shared_ptr<HyperGraph::Edge>& e) override {
        if (! vParent)
          return 0.;
        EdgeSE2* odom    = static_cast<EdgeSE2*>(e.get());
        VertexSE2* from  = static_cast<VertexSE2*>(vParent.get());
        VertexSE2* to    = static_cast<VertexSE2*>(v.get());
        assert(to->hessianIndex() >= 0);
        number_t fromTheta = from->hessianIndex() < 0 ? 0. : thetaGuess_[from->hessianIndex()];
        bool direct      = odom->vertices()[0].get() == from;
        if (direct)
          thetaGuess_[to->hessianIndex()] = fromTheta + odom->measurement().rotation().angle();
        else
          thetaGuess_[to->hessianIndex()] = fromTheta - odom->measurement().rotation().angle();
        return 1.;
      }

    protected:
      VectorX& thetaGuess_;
  };

  SolverSLAM2DLinear::SolverSLAM2DLinear(std::unique_ptr<Solver> solver)
    : OptimizationAlgorithmGaussNewton(std::move(solver))
  {}

  SolverSLAM2DLinear::~SolverSLAM2DLinear()
  {}

  OptimizationAlgorithm::SolverResult SolverSLAM2DLinear::solve(int iteration, bool online)
  {
    if (iteration == 0) {
      bool status = solveOrientation();
      if (! status)
        return OptimizationAlgorithm::kFail;
    }

    return OptimizationAlgorithmGaussNewton::solve(iteration, online);
  }

  bool SolverSLAM2DLinear::solveOrientation()
  {
    assert(_optimizer->indexMapping().size() + 1 == _optimizer->vertices().size() && "Needs to operate on full graph");
    assert(_optimizer->vertex(0)->fixed() && "Graph is not fixed by vertex 0");
    VectorX b, x; // will be used for theta and x/y update
    b.setZero(optimizer_->indexMapping().size());
    x.setZero(optimizer_->indexMapping().size());

    using ScalarMatrix = int;

    std::vector<int> blockIndeces(_optimizer->indexMapping().size());
    for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i)
      blockIndeces[i] = i+1;

    SparseBlockMatrix<ScalarMatrix> H(blockIndeces.data(), blockIndeces.data(), _optimizer->indexMapping().size(), _optimizer->indexMapping().size());

    // building the structure, diagonal for each active vertex
    for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
      OptimizableGraph::Vertex* v = optimizer_->indexMapping()[i];
      int poseIdx = v->hessianIndex();
      ScalarMatrix* m = H.block(poseIdx, poseIdx, true);
      m->setZero();
    }

    HyperGraph::VertexSet fixedSet;

    // off diagonal for each edge
    for (SparseOptimizer::EdgeContainer::const_iterator it = optimizer_->activeEdges().begin(); it != optimizer_->activeEdges().end(); ++it) {
#    ifndef NDEBUG
      EdgeSE2* e = dynamic_cast<EdgeSE2*>(it->get());
      assert(e && "Active edges contain non-odometry edge"); //
#    else
      EdgeSE2* e = static_cast<EdgeSE2*>(it->get());
#    endif
      auto from = e->vertexXn<0>();
      auto to   = e->vertexXn<1>();

      int ind1 = from->hessianIndex();
      int ind2 = to->hessianIndex();
      if (ind1 == -1 || ind2 == -1) {
        if (ind1 == -1) fixedSet.insert(from); // collect the fixed vertices
        if (ind2 == -1) fixedSet.insert(to);
        continue;
      }

      bool transposedBlock = ind1 > ind2;
      if (transposedBlock){ // make sure, we allocate the upper triangle block
        std::swap(ind1, ind2);
      }

      ScalarMatrix* m = H.block(ind1, ind2, true);
      m->setZero();
    }

    // walk along the Minimal Spanning Tree to compute the guess for the robot orientation
    assert(fixedSet.size() == 1);
    auto root = static_pointer_cast<VertexSE2>(*fixedSet.begin());
    VectorX thetaGuess;
    thetaGuess.setZero(optimizer_->indexMapping().size());
    UniformCostFunction uniformCost;
    auto pointerWrapper = std::shared_ptr<HyperGraph>(optimizer_, [](HyperGraph*){});
    HyperDijkstra hyperDijkstra(pointerWrapper);
    hyperDijkstra.shortestPaths(root, &uniformCost);

    HyperDijkstra::computeTree(hyperDijkstra.adjacencyMap());
    ThetaTreeAction thetaTreeAction(thetaGuess);
    HyperDijkstra::visitAdjacencyMap(hyperDijkstra.adjacencyMap(), &thetaTreeAction);

    // construct for the orientation
    for (SparseOptimizer::EdgeContainer::const_iterator it = optimizer_->activeEdges().begin(); it != optimizer_->activeEdges().end(); ++it) {
      EdgeSE2* e = static_cast<EdgeSE2*>(it->get());
      auto from = e->vertexXn<0>();
      auto to   = e->vertexXn<1>();

      number_t omega = e->information()(2,2);

      number_t fromThetaGuess = from->hessianIndex() < 0 ? 0. : thetaGuess[from->hessianIndex()];
      number_t toThetaGuess   = to->hessianIndex() < 0 ? 0. : thetaGuess[to->hessianIndex()];
      number_t error          = normalize_theta(-e->measurement().rotation().angle() + toThetaGuess - fromThetaGuess);

      bool fromNotFixed = !(from->fixed());
      bool toNotFixed   = !(to->fixed());

      if (fromNotFixed || toNotFixed) {
        number_t omega_r = - omega * error;
        if (fromNotFixed) {
          b(from->hessianIndex()) -= omega_r;
          (*H.block(from->hessianIndex(), from->hessianIndex()))(0,0) += omega;
          if (toNotFixed) {
            if (from->hessianIndex() > to->hessianIndex())
              (*H.block(to->hessianIndex(), from->hessianIndex()))(0,0) -= omega;
            else
              (*H.block(from->hessianIndex(), to->hessianIndex()))(0,0) -= omega;
          }
        }
        if (toNotFixed ) {
          b(to->hessianIndex()) += omega_r;
          (*H.block(to->hessianIndex(), to->hessianIndex()))(0,0) += omega;
        }
      }
    }

    // solve orientation
    using SystemSolver = LinearSolverEigen<ScalarMatrix>;
    SystemSolver linearSystemSolver;
    linearSystemSolver.init();
    bool ok = linearSystemSolver.solve(H, x.data(), b.data());
    if (!ok) {
      cerr << __PRETTY_FUNCTION__ << "Failure while solving linear system" << endl;
      return false;
    }

    // update the orientation of the 2D poses and set translation to 0, GN shall solve that
    root->setToOrigin();
    for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
      VertexSE2* v = static_cast<VertexSE2*>(optimizer_->indexMapping()[i]);
      int poseIdx = v->hessianIndex();
      SE2 poseUpdate(0, 0, normalize_theta(thetaGuess(poseIdx) + x(poseIdx)));
      v->setEstimate(poseUpdate);
    }

    return true;
  }

} // end namespace
