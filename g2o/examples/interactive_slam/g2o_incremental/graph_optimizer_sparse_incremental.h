// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H
#define G2O_GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/examples/interactive_slam/g2o_interactive/graph_optimizer_sparse_online.h"
#include "g2o_incremental_api.h"
#include "linear_solver_cholmod_online.h"

namespace g2o {

namespace cholmod {
struct CholmodExt;
}

class G2O_INCREMENTAL_API SparseOptimizerIncremental
    : public SparseOptimizerOnline {
 public:
  SparseOptimizerIncremental();
  ~SparseOptimizerIncremental() override;

  int optimize(int iterations, bool online = false) override;

  bool updateInitialization(HyperGraph::VertexSet& vset,
                            HyperGraph::EdgeSet& eset) override;

  bool initSolver(int dimension, int batchEveryN) override;

 protected:
  SparseBlockMatrix<Eigen::MatrixXd> updateMat_;
  cholmod_common cholmodCommon_;
  cholmod::CholmodExt* cholmodSparse_;
  cholmod_factor* cholmodFactor_;
  cholmod_triplet* permutedUpdate_;
  cholmod_factor* L_;
  LinearSolverCholmodOnlineInterface* solverInterface_;

  HyperGraph::VertexSet touchedVertices_;
  Eigen::VectorXi perm_;
  Eigen::VectorXi cmember_;

  Eigen::VectorXi tripletWorkspace_;
  cholmod::CholmodExt* permutedUpdateAsSparse_;

  bool computeCholeskyUpdate();
  void convertTripletUpdateToSparse();
};

}  // namespace g2o

#endif
