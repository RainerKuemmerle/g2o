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

#include "solver_slam2d_linear.h"

#include <Eigen/Core>

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/stuff/misc.h"
#include "g2o/stuff/scoped_pointer.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"
using namespace std;

namespace g2o {

  /**
   * \brief compute the initial guess of theta while travelling along the MST
   */
  class ThetaTreeAction : public HyperDijkstra::TreeAction
  {
    public:
      ThetaTreeAction(double* theta) : HyperDijkstra::TreeAction(), _thetaGuess(theta) {}
      virtual double perform(HyperGraph::Vertex* v, HyperGraph::Vertex* vParent, HyperGraph::Edge* e)
      {
        if (! vParent)
          return 0.;
        EdgeSE2* odom    = static_cast<EdgeSE2*>(e);
        VertexSE2* from  = static_cast<VertexSE2*>(vParent);
        VertexSE2* to    = static_cast<VertexSE2*>(v);
        assert(to->tempIndex() >= 0);
        double fromTheta = from->tempIndex() < 0 ? 0. : _thetaGuess[from->tempIndex()];
        bool direct      = odom->vertices()[0] == from;
        if (direct) 
          _thetaGuess[to->tempIndex()] = fromTheta + odom->measurement().rotation().angle();
        else
          _thetaGuess[to->tempIndex()] = fromTheta - odom->measurement().rotation().angle();
        return 1.;
      }
    protected:
      double* _thetaGuess;
  };

  SolverSLAM2DLinear::SolverSLAM2DLinear(Solver* solver) :
    OptimizationAlgorithmGaussNewton(solver)
  {
  }

  SolverSLAM2DLinear::~SolverSLAM2DLinear()
  {
  }

  OptimizationAlgorithm::SolverResult SolverSLAM2DLinear::solve(int iteration, bool online)
  {
    if (iteration == 0) {
      bool status = solveOrientation();
      if (! status)
        return OptimizationAlgorithm::Fail;
    }

    return OptimizationAlgorithmGaussNewton::solve(iteration, online);
  }

  bool SolverSLAM2DLinear::solveOrientation()
  {
    assert(_optimizer->indexMapping().size() + 1 == _optimizer->vertices().size() && "Needs to operate on full graph");
    assert(_optimizer->vertex(0)->fixed() && "Graph is not fixed by vertex 0");
    Eigen::VectorXd b, x; // will be used for theta and x/y update
    b.setZero(_optimizer->indexMapping().size());
    x.setZero(_optimizer->indexMapping().size());

    typedef Eigen::Matrix<double, 1, 1> ScalarMatrix;

    ScopedArray<int> blockIndeces(new int[_optimizer->indexMapping().size()]);
    for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i)
      blockIndeces[i] = i+1;

    SparseBlockMatrix<ScalarMatrix> H(blockIndeces.get(), blockIndeces.get(), _optimizer->indexMapping().size(), _optimizer->indexMapping().size());

    // building the structure, diagonal for each active vertex
    for (size_t i = 0; i < _optimizer->indexMapping().size(); ++i) {
      OptimizableGraph::Vertex* v = _optimizer->indexMapping()[i];
      int poseIdx = v->tempIndex();
      ScalarMatrix* m = H.block(poseIdx, poseIdx, true);
      m->setZero();
    }

    HyperGraph::VertexSet fixedSet;

    // off diagonal for each edge
    for (SparseOptimizer::EdgeContainer::const_iterator it = _optimizer->activeEdges().begin(); it != _optimizer->activeEdges().end(); ++it) {
#    ifndef NDEBUG
      EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
      assert(e && "Active edges contain non-odometry edge"); //
#    else
      EdgeSE2* e = static_cast<EdgeSE2*>(*it);
#    endif
      OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
      OptimizableGraph::Vertex* to   = static_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);

      int ind1 = from->tempIndex();
      int ind2 = to->tempIndex();
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
    VertexSE2* root = static_cast<VertexSE2*>(*fixedSet.begin());
    VectorXd thetaGuess;
    thetaGuess.setZero(_optimizer->indexMapping().size());
    UniformCostFunction uniformCost;
    HyperDijkstra hyperDijkstra(_optimizer);
    hyperDijkstra.shortestPaths(root, &uniformCost);

    HyperDijkstra::computeTree(hyperDijkstra.adjacencyMap());
    ThetaTreeAction thetaTreeAction(thetaGuess.data());
    HyperDijkstra::visitAdjacencyMap(hyperDijkstra.adjacencyMap(), &thetaTreeAction);

    // construct for the orientation
    for (SparseOptimizer::EdgeContainer::const_iterator it = _optimizer->activeEdges().begin(); it != _optimizer->activeEdges().end(); ++it) {
      EdgeSE2* e = static_cast<EdgeSE2*>(*it);
      VertexSE2* from = static_cast<VertexSE2*>(e->vertices()[0]);
      VertexSE2* to   = static_cast<VertexSE2*>(e->vertices()[1]);

      double omega = e->information()(2,2);

      double fromThetaGuess = from->tempIndex() < 0 ? 0. : thetaGuess[from->tempIndex()];
      double toThetaGuess   = to->tempIndex() < 0 ? 0. : thetaGuess[to->tempIndex()];
      double error          = normalize_theta(-e->measurement().rotation().angle() + toThetaGuess - fromThetaGuess);

      bool fromNotFixed = !(from->fixed());
      bool toNotFixed   = !(to->fixed());

      if (fromNotFixed || toNotFixed) {
        double omega_r = - omega * error;
        if (fromNotFixed) {
          b(from->tempIndex()) -= omega_r;
          (*H.block(from->tempIndex(), from->tempIndex()))(0,0) += omega;
          if (toNotFixed) {
            if (from->tempIndex() > to->tempIndex())
              (*H.block(to->tempIndex(), from->tempIndex()))(0,0) -= omega;
            else
              (*H.block(from->tempIndex(), to->tempIndex()))(0,0) -= omega;
          }
        } 
        if (toNotFixed ) {
          b(to->tempIndex()) += omega_r;
          (*H.block(to->tempIndex(), to->tempIndex()))(0,0) += omega;
        }
      }
    }

    // solve orientation
    typedef LinearSolverCSparse<ScalarMatrix> SystemSolver;
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
      VertexSE2* v = static_cast<VertexSE2*>(_optimizer->indexMapping()[i]);
      int poseIdx = v->tempIndex();
      SE2 poseUpdate(0, 0, normalize_theta(thetaGuess(poseIdx) + x(poseIdx)));
      v->setEstimate(poseUpdate);
    }

    return true;
  }

} // end namespace
