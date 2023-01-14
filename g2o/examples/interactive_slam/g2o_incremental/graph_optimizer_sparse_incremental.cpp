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

#include "graph_optimizer_sparse_incremental.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/examples/interactive_slam/g2o_interactive/types_slam2d_online.h"
#include "g2o/examples/interactive_slam/g2o_interactive/types_slam3d_online.h"
#include "g2o/examples/target/targetTypes6D.hpp"
#include "g2o/stuff/macros.h"

namespace g2o {

namespace {

template <int P, int L>
std::unique_ptr<g2o::Solver> AllocateCholmodSolver() {
  std::cerr << "# Using CHOLMOD online poseDim " << P << " landMarkDim " << L
            << " blockordering 1" << std::endl;

  return g2o::make_unique<BlockSolverPL<P, L>>(
      g2o::make_unique<LinearSolverCholmodOnline<
          typename BlockSolverPL<P, L>::PoseMatrixType>>());
}

/**
 * \brief backing up some information about the vertex
 */
struct VertexBackup {
  int hessianIndex;
  OptimizableGraph::Vertex* vertex;
  double* hessianData;
  bool operator<(const VertexBackup& other) const {
    return hessianIndex < other.hessianIndex;
  }
};
}  // namespace

SparseOptimizerIncremental::SparseOptimizerIncremental() {
  cholmodSparse_ = new cholmod::CholmodExt();
  cholmodFactor_ = nullptr;
  cholmod_start(&cholmodCommon_);

  // setup ordering strategy to not permute the matrix
  cholmodCommon_.nmethods = 1;
  cholmodCommon_.method[0].ordering = CHOLMOD_NATURAL;
  cholmodCommon_.postorder = 0;
  cholmodCommon_.supernodal = CHOLMOD_SIMPLICIAL;

  permutedUpdate_ = cholmod_allocate_triplet(1000, 1000, 1024, 0, CHOLMOD_REAL,
                                             &cholmodCommon_);
  L_ = nullptr;
  cholmodFactor_ = nullptr;
  solverInterface_ = nullptr;

  permutedUpdateAsSparse_ = new cholmod::CholmodExt;
}

SparseOptimizerIncremental::~SparseOptimizerIncremental() {
  delete permutedUpdateAsSparse_;
  updateMat_.clear(true);
  if (cholmodFactor_) {
    cholmod_free_factor(&cholmodFactor_, &cholmodCommon_);
    cholmodFactor_ = nullptr;
  }
  cholmod_free_triplet(&permutedUpdate_, &cholmodCommon_);
  cholmod_finish(&cholmodCommon_);
}

int SparseOptimizerIncremental::optimize(int iterations, bool online) {
  (void)iterations;  // we only do one iteration anyhow
  algorithm_->init(online);

  bool ok = true;

  if (!online || batchStep) {
    // cerr << "performing batch step" << endl;
    if (!online) {
      ok = underlyingSolver_->buildStructure();
      if (!ok) {
        std::cerr << __PRETTY_FUNCTION__
                  << ": Failure while building CCS structure" << std::endl;
        return 0;
      }
    }

    // copy over the updated estimate as new linearization point
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

    // mark vertices to be sorted as last
    const int numBlocksRequired = ivMap_.size();
    if (cmember_.size() < numBlocksRequired) {
      cmember_.resize(static_cast<Eigen::Index>(2) * numBlocksRequired);
    }
    memset(cmember_.data(), 0, numBlocksRequired * sizeof(int));
    if (ivMap_.size() > 100) {
      for (size_t i = ivMap_.size() - 20; i < ivMap_.size(); ++i) {
        const HyperGraph::EdgeSetWeak& eset = ivMap_[i]->edges();
        for (const auto& it : eset) {
          auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it.lock());
          auto* v1 =
              static_cast<OptimizableGraph::Vertex*>(e->vertices()[0].get());
          auto* v2 =
              static_cast<OptimizableGraph::Vertex*>(e->vertices()[1].get());
          if (v1->hessianIndex() >= 0) cmember_(v1->hessianIndex()) = 1;
          if (v2->hessianIndex() >= 0) cmember_(v2->hessianIndex()) = 1;
        }
      }
      // OptimizableGraph::Vertex* lastPose = ivMap_.back();
      //_cmember(lastPose->hessianIndex()) = 2;
    }

    ok = underlyingSolver_->solve();

    // get the current cholesky factor along with the permutation
    L_ = solverInterface_->L();
    if (perm_.size() < static_cast<int>(L_->n)) perm_.resize(2 * L_->n);
    int* p = static_cast<int*>(L_->Perm);
    for (size_t i = 0; i < L_->n; ++i) perm_[p[i]] = i;

  } else {
    // update the b vector
    for (const auto& _touchedVertice : touchedVertices_) {
      auto* v = static_cast<OptimizableGraph::Vertex*>(_touchedVertice.get());
      const int iBase = v->colInHessian();
      v->copyB(underlyingSolver_->b() + iBase);
    }
    solverInterface_->solve(underlyingSolver_->x(), underlyingSolver_->b());
  }

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

bool SparseOptimizerIncremental::updateInitialization(
    HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset) {
  if (batchStep) {
    return SparseOptimizerOnline::updateInitialization(vset, eset);
  }

  for (const auto& it : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    v->clearQuadraticForm();  // be sure that b is zero for this vertex
  }

  // get the touched vertices
  touchedVertices_.clear();
  for (const auto& it : eset) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    auto v1 =
        std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[0]);
    auto v2 =
        std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertices()[1]);
    if (!v1->fixed()) touchedVertices_.insert(v1);
    if (!v2->fixed()) touchedVertices_.insert(v2);
  }
  // cerr << PVAR(_touchedVertices.size()) << endl;

  // updating the internal structures
  HyperGraph::VertexContainer newVertices;
  newVertices.reserve(vset.size());
  activeVertices_.reserve(activeVertices_.size() + vset.size());
  activeEdges_.reserve(activeEdges_.size() + eset.size());
  for (const auto& it : eset)
    activeEdges_.push_back(
        std::static_pointer_cast<OptimizableGraph::Edge>(it));
  // cerr << "updating internal done." << endl;

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
  // cerr << "updating index mapping done." << endl;

  // backup the tempindex and prepare sorting structure
#ifdef _MSC_VER
  VertexBackup* backupIdx = new VertexBackup[_touchedVertices.size()];
#else
  VertexBackup backupIdx[touchedVertices_.size()];
#endif
  memset(backupIdx, 0, sizeof(VertexBackup) * touchedVertices_.size());
  int idx = 0;
  for (const auto& _touchedVertice : touchedVertices_) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(_touchedVertice.get());
    backupIdx[idx].hessianIndex = v->hessianIndex();
    backupIdx[idx].vertex = v;
    backupIdx[idx].hessianData = v->hessianData();
    ++idx;
  }
  // sort according to the hessianIndex which is the same order as used later by
  // the optimizer
  std::sort(backupIdx, backupIdx + touchedVertices_.size());
  for (int i = 0; i < idx; ++i) {
    backupIdx[i].vertex->setHessianIndex(i);
  }
  // cerr << "backup tempindex done." << endl;

  // building the structure of the update
  updateMat_.clear(true);  // get rid of the old matrix structure
  updateMat_.rowBlockIndices().clear();
  updateMat_.colBlockIndices().clear();
  updateMat_.blockCols().clear();

  // placing the current stuff in _updateMat
  MatrixX* lastBlock = nullptr;
  int sizePoses = 0;
  for (int i = 0; i < idx; ++i) {
    OptimizableGraph::Vertex* v = backupIdx[i].vertex;
    const int dim = v->dimension();
    sizePoses += dim;
    updateMat_.rowBlockIndices().push_back(sizePoses);
    updateMat_.colBlockIndices().push_back(sizePoses);
    updateMat_.blockCols().emplace_back();
    const int ind = v->hessianIndex();
    // cerr << PVAR(ind) << endl;
    if (ind >= 0) {
      MatrixX* m = updateMat_.block(ind, ind, true);
      v->mapHessianMemory(m->data());
      lastBlock = m;
    }
  }
  lastBlock->diagonal().array() += 1e-6;  // HACK to get Eigen value > 0

  for (const auto& it : eset) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    auto* v1 = static_cast<OptimizableGraph::Vertex*>(e->vertices()[0].get());
    auto* v2 = static_cast<OptimizableGraph::Vertex*>(e->vertices()[1].get());

    int ind1 = v1->hessianIndex();
    if (ind1 == -1) continue;
    int ind2 = v2->hessianIndex();
    if (ind2 == -1) continue;
    const bool transposedBlock = ind1 > ind2;
    if (transposedBlock)  // make sure, we allocate the upper triangular block
      std::swap(ind1, ind2);

    Eigen::MatrixXd* m = updateMat_.block(ind1, ind2, true);
    e->mapHessianMemory(m->data(), 0, 1, transposedBlock);
  }

  // build the system into _updateMat
  for (const auto& it : eset) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    e->computeError();
  }
  for (const auto& it : eset) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    e->linearizeOplus(jacobianWorkspace());
    e->constructQuadraticForm();
  }

  // restore the original data for the vertex
  for (int i = 0; i < idx; ++i) {
    backupIdx[i].vertex->setHessianIndex(backupIdx[i].hessianIndex);
    if (backupIdx[i].hessianData)
      backupIdx[i].vertex->mapHessianMemory(backupIdx[i].hessianData);
  }

  // update the structure of the real block matrix
  const bool solverStatus = algorithm_->updateStructure(newVertices, eset);

  const bool updateStatus = computeCholeskyUpdate();
  if (!updateStatus) {
    std::cerr << "Error while computing update" << std::endl;
  }

  cholmod_sparse* updateAsSparseFactor =
      cholmod_factor_to_sparse(cholmodFactor_, &cholmodCommon_);

  // convert CCS update by permuting back to the permutation of L
  if (updateAsSparseFactor->nzmax > permutedUpdate_->nzmax) {
    // cerr << "realloc _permutedUpdate" << endl;
    cholmod_reallocate_triplet(updateAsSparseFactor->nzmax, permutedUpdate_,
                               &cholmodCommon_);
  }
  permutedUpdate_->nnz = 0;
  permutedUpdate_->nrow = permutedUpdate_->ncol = L_->n;
  {
    int* Ap = static_cast<int*>(updateAsSparseFactor->p);
    int* Ai = static_cast<int*>(updateAsSparseFactor->i);
    auto* Ax = static_cast<double*>(updateAsSparseFactor->x);
    int* Bj = static_cast<int*>(permutedUpdate_->j);
    int* Bi = static_cast<int*>(permutedUpdate_->i);
    auto* Bx = static_cast<double*>(permutedUpdate_->x);
    for (size_t c = 0; c < updateAsSparseFactor->ncol; ++c) {
      const int& rbeg = Ap[c];
      const int& rend = Ap[c + 1];
      const int cc = c / slamDimension;
      const int coff = c % slamDimension;
      const int& cbase = backupIdx[cc].vertex->colInHessian();
      const int& ccol = perm_(cbase + coff);
      for (int j = rbeg; j < rend; j++) {
        const int& r = Ai[j];
        const double& val = Ax[j];

        const int rr = r / slamDimension;
        const int roff = r % slamDimension;
        const int& rbase = backupIdx[rr].vertex->colInHessian();

        int row = perm_(rbase + roff);
        int col = ccol;
        if (col > row)  // lower triangular entry
          std::swap(col, row);
        Bi[permutedUpdate_->nnz] = row;
        Bj[permutedUpdate_->nnz] = col;
        Bx[permutedUpdate_->nnz] = val;
        ++permutedUpdate_->nnz;
      }
    }
  }
  cholmod_free_sparse(&updateAsSparseFactor, &cholmodCommon_);
#ifdef _MSC_VER
  delete[] backupIdx;
#endif

#if 0
    cholmod_sparse* updatePermuted = cholmod_triplet_to_sparse(_permutedUpdate, _permutedUpdate->nnz, &cholmodCommon_);
    //writeCCSMatrix("update-permuted.txt", updatePermuted->nrow, updatePermuted->ncol, (int*)updatePermuted->p, (int*)updatePermuted->i, (double*)updatePermuted->x, false);
    _solverInterface->choleskyUpdate(updatePermuted);
    cholmod_free_sparse(&updatePermuted, &cholmodCommon_);
#else
  convertTripletUpdateToSparse();
  solverInterface_->choleskyUpdate(permutedUpdateAsSparse_);
#endif

  return solverStatus;
}

bool SparseOptimizerIncremental::computeCholeskyUpdate() {
  if (cholmodFactor_) {
    cholmod_free_factor(&cholmodFactor_, &cholmodCommon_);
    cholmodFactor_ = nullptr;
  }

  const SparseBlockMatrix<MatrixX>& A = updateMat_;
  const size_t m = A.rows();
  const size_t n = A.cols();

  if (cholmodSparse_->columnsAllocated < n) {
    // std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" <<
    // std::endl;
    cholmodSparse_->columnsAllocated =
        cholmodSparse_->columnsAllocated == 0
            ? n
            : 2 * n;  // pre-allocate more space if re-allocating
    delete[] static_cast<int*>(cholmodSparse_->p);
    cholmodSparse_->p = new int[cholmodSparse_->columnsAllocated + 1];
  }
  const size_t nzmax = A.nonZeros();
  if (cholmodSparse_->nzmax < nzmax) {
    // std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" <<
    // std::endl;
    cholmodSparse_->nzmax =
        cholmodSparse_->nzmax == 0
            ? nzmax
            : 2 * nzmax;  // pre-allocate more space if re-allocating
    delete[] static_cast<double*>(cholmodSparse_->x);
    delete[] static_cast<int*>(cholmodSparse_->i);
    cholmodSparse_->i = new int[cholmodSparse_->nzmax];
    cholmodSparse_->x = new double[cholmodSparse_->nzmax];
  }
  cholmodSparse_->ncol = n;
  cholmodSparse_->nrow = m;

  A.fillCCS(static_cast<int*>(cholmodSparse_->p),
            static_cast<int*>(cholmodSparse_->i),
            static_cast<double*>(cholmodSparse_->x), true);
  // writeCCSMatrix("updatesparse.txt", cholmodSparse_->nrow,
  // cholmodSparse_->ncol, (int*)cholmodSparse_->,_ (int*)cholmodSparse_->i,
  // (double*)holmodSparse__->x, rue_;_

  cholmodFactor_ = cholmod_analyze(cholmodSparse_, &cholmodCommon_);
  cholmod_factorize(cholmodSparse_, cholmodFactor_, &cholmodCommon_);

#if 0
    int* p = (int*)_cholmodFactor->Perm;
    for (int i = 0; i < (int)n; ++i)
      if (i != p[i])
        cerr << "wrong permutation" << i << " -> " << p[i] << endl;
#endif

  if (cholmodCommon_.status == CHOLMOD_NOT_POSDEF) {
    // std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by
    // Octave)" << std::endl; writeCCSMatrix("debug.txt", cholmodSparse_->nrow,
    // cholmodSparse_->ncol, (int*)cholmodSparse_->p, (int*)holmodSparse__->i,
    // (double*)holmodSparse__->x, rue_);
    return false;
  }

  // change to the specific format we need to have a pretty normal L
  const int change_status = cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1,
                                            cholmodFactor_, &cholmodCommon_);
  return change_status != 0;
}

static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& solverName) {
  std::unique_ptr<g2o::Solver> s;

  if (solverName == "fix3_2_cholmod") {
    s = AllocateCholmodSolver<3, 2>();
  } else if (solverName == "fix6_3_cholmod") {
    s = AllocateCholmodSolver<6, 3>();
  }

  std::unique_ptr<OptimizationAlgorithm> gaussNewton(
      new OptimizationAlgorithmGaussNewton(std::move(s)));
  return gaussNewton;
}

bool SparseOptimizerIncremental::initSolver(int dimension, int batchEveryN) {
  // cerr << __PRETTY_FUNCTION__ << endl;
  slamDimension = dimension;
  if (dimension == 3) {
    setAlgorithm(createSolver("fix3_2_cholmod"));
    OptimizationAlgorithmGaussNewton* gaussNewton =
        dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver().get());
    assert(gaussNewton);
    auto* bs = dynamic_cast<BlockSolver<BlockSolverTraits<3, 2>>*>(
        &gaussNewton->solver());
    assert(bs && "Unable to get internal block solver");
    auto* s =
        dynamic_cast<LinearSolverCholmodOnline<Matrix3>*>(&bs->linearSolver());
    bs->setAdditionalVectorSpace(300);
    bs->setSchur(false);
    solverInterface_ = s;
    underlyingSolver_ = bs;
  } else {
    setAlgorithm(createSolver("fix6_3_cholmod"));
    OptimizationAlgorithmGaussNewton* gaussNewton =
        dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver().get());
    assert(gaussNewton);
    auto* bs = dynamic_cast<BlockSolver<BlockSolverTraits<6, 3>>*>(
        &gaussNewton->solver());
    assert(bs && "Unable to get internal block solver");
    auto* s =
        dynamic_cast<LinearSolverCholmodOnline<Matrix6d>*>(&bs->linearSolver());
    bs->setAdditionalVectorSpace(600);
    bs->setSchur(false);
    solverInterface_ = s;
    underlyingSolver_ = bs;
  }
  solverInterface_->cmember = &cmember_;
  solverInterface_->batchEveryN = batchEveryN;
  if (!solver()) {
    std::cerr << "Error allocating solver. Allocating CHOLMOD solver failed!"
              << std::endl;
    return false;
  }
  return true;
}

void SparseOptimizerIncremental::convertTripletUpdateToSparse() {
  // re-allocate the memory
  if (tripletWorkspace_.size() < static_cast<int>(permutedUpdate_->ncol)) {
    tripletWorkspace_.resize(permutedUpdate_->ncol * 2);
  }

  // reallocate num-zeros
  if (permutedUpdateAsSparse_->nzmax < permutedUpdate_->nzmax) {
    permutedUpdateAsSparse_->nzmax = permutedUpdate_->nzmax;
    delete[] static_cast<int*>(permutedUpdateAsSparse_->i);
    delete[] static_cast<double*>(permutedUpdateAsSparse_->x);
    permutedUpdateAsSparse_->x = new double[permutedUpdateAsSparse_->nzmax];
    permutedUpdateAsSparse_->i = new int[permutedUpdateAsSparse_->nzmax];
  }

  if (permutedUpdateAsSparse_->columnsAllocated < permutedUpdate_->ncol) {
    permutedUpdateAsSparse_->columnsAllocated = 2 * permutedUpdate_->ncol;
    delete[] static_cast<int*>(permutedUpdateAsSparse_->p);
    permutedUpdateAsSparse_->p =
        new int[permutedUpdateAsSparse_->columnsAllocated + 1];
  }

  permutedUpdateAsSparse_->ncol = permutedUpdate_->ncol;
  permutedUpdateAsSparse_->nrow = permutedUpdate_->nrow;

  int* w = tripletWorkspace_.data();
  memset(w, 0, sizeof(int) * permutedUpdate_->ncol);

  int* Ti = static_cast<int*>(permutedUpdate_->i);
  int* Tj = static_cast<int*>(permutedUpdate_->j);
  auto* Tx = static_cast<double*>(permutedUpdate_->x);

  int* Cp = static_cast<int*>(permutedUpdateAsSparse_->p);
  int* Ci = static_cast<int*>(permutedUpdateAsSparse_->i);
  auto* Cx = static_cast<double*>(permutedUpdateAsSparse_->x);

  for (size_t k = 0; k < permutedUpdate_->nnz; ++k) /* column counts */
    w[Tj[k]]++;

  /* column pointers */
  const int n = permutedUpdate_->ncol;
  int nz = 0;
  for (int i = 0; i < n; i++) {
    Cp[i] = nz;
    nz += w[i];
    w[i] = Cp[i];
  }
  Cp[n] = nz;
  assert((size_t)nz == permutedUpdate_->nnz);

  for (size_t k = 0; k < permutedUpdate_->nnz; ++k) {
    const int p = w[Tj[k]]++;
    Ci[p] = Ti[k]; /* A(i,j) is the pth entry in C */
    Cx[p] = Tx[k];
  }
}

}  // namespace g2o
