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

#ifndef G2O_BLOCK_SOLVER_H
#define G2O_BLOCK_SOLVER_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dynamic_aligned_buffer.hpp"
#include "g2o/config.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_block_matrix_ccs.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/timeutil.h"
#include "linear_solver.h"
#include "solver.h"
#include "sparse_block_matrix.h"
#include "sparse_block_matrix_diagonal.h"  // IWYU pragma: keep
#include "sparse_optimizer.h"

#ifdef G2O_OPENMP
#include "openmp_mutex.h"  // IWYU pragma: keep
#endif

namespace g2o {

/**
 * \brief traits to summarize the properties of the fixed size optimization
 * problem
 */
template <int PoseDim, int LandmarkDim>
struct BlockSolverTraits {
  static const int kPoseDim = PoseDim;
  static const int kLandmarkDim = LandmarkDim;
  using PoseMatrixType =
      Eigen::Matrix<double, kPoseDim, kPoseDim, Eigen::ColMajor>;
  using LandmarkMatrixType =
      Eigen::Matrix<double, kLandmarkDim, kLandmarkDim, Eigen::ColMajor>;
  using PoseLandmarkMatrixType =
      Eigen::Matrix<double, kPoseDim, kLandmarkDim, Eigen::ColMajor>;
  using PoseVectorType = Eigen::Matrix<double, kPoseDim, 1, Eigen::ColMajor>;
  using LandmarkVectorType =
      Eigen::Matrix<double, kLandmarkDim, 1, Eigen::ColMajor>;

  using PoseHessianType = SparseBlockMatrix<PoseMatrixType>;
  using LandmarkHessianType = SparseBlockMatrix<LandmarkMatrixType>;
  using PoseLandmarkHessianType = SparseBlockMatrix<PoseLandmarkMatrixType>;
  using LinearSolverType = LinearSolver<PoseMatrixType>;
};

/**
 * \brief traits to summarize the properties of the dynamic size optimization
 * problem
 */
template <>
struct BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> {
  static const int kPoseDim = Eigen::Dynamic;
  static const int kLandmarkDim = Eigen::Dynamic;
  using PoseMatrixType = MatrixX;
  using LandmarkMatrixType = MatrixX;
  using PoseLandmarkMatrixType = MatrixX;
  using PoseVectorType = VectorX;
  using LandmarkVectorType = VectorX;

  using PoseHessianType = SparseBlockMatrix<PoseMatrixType>;
  using LandmarkHessianType = SparseBlockMatrix<LandmarkMatrixType>;
  using PoseLandmarkHessianType = SparseBlockMatrix<PoseLandmarkMatrixType>;
  using LinearSolverType = LinearSolver<PoseMatrixType>;
};

/**
 * \brief base for the block solvers with some basic function interfaces
 */
class BlockSolverBase : public Solver {
 public:
  ~BlockSolverBase() override = default;
  /**
   * compute dest = H * src
   */
  virtual void multiplyHessian(double* dest, const double* src) const = 0;
};

/**
 * \brief Implementation of a solver operating on the blocks of the Hessian
 */
template <typename Traits>
class BlockSolver : public BlockSolverBase {
 public:
  static const int kPoseDim = Traits::kPoseDim;
  static const int kLandmarkDim = Traits::kLandmarkDim;
  using PoseMatrixType = typename Traits::PoseMatrixType;
  using LandmarkMatrixType = typename Traits::LandmarkMatrixType;
  using PoseLandmarkMatrixType = typename Traits::PoseLandmarkMatrixType;
  using PoseVectorType = typename Traits::PoseVectorType;
  using LandmarkVectorType = typename Traits::LandmarkVectorType;

  using PoseHessianType = typename Traits::PoseHessianType;
  using LandmarkHessianType = typename Traits::LandmarkHessianType;
  using PoseLandmarkHessianType = typename Traits::PoseLandmarkHessianType;
  using LinearSolverType = typename Traits::LinearSolverType;

  /**
   * allocate a block solver ontop of the underlying linear solver.
   * NOTE: The BlockSolver assumes exclusive access to the linear solver and
   * will therefore free the pointer in its destructor.
   */
  explicit BlockSolver(std::unique_ptr<LinearSolverType> linearSolver);
  ~BlockSolver() override;

  bool init(SparseOptimizer* optimizer, bool online = false) override;
  bool buildStructure(bool zeroBlocks = false) override;
  bool updateStructure(const HyperGraph::VertexContainer& vset,
                       const HyperGraph::EdgeSet& edges) override;
  bool buildSystem() override;
  bool solve() override;
  bool computeMarginals(
      SparseBlockMatrix<MatrixX>& spinv,
      const std::vector<std::pair<int, int>>& blockIndices) override;
  bool setLambda(double lambda, bool backup = false) override;
  void restoreDiagonal() override;
  bool supportsSchur() override { return true; }
  bool schur() override { return doSchur_; }
  void setSchur(bool s) override { doSchur_ = s; }

  LinearSolver<PoseMatrixType>& linearSolver() const { return *linearSolver_; }

  void setWriteDebug(bool writeDebug) override;
  [[nodiscard]] bool writeDebug() const override {
    return linearSolver_->writeDebug();
  }

  [[nodiscard]] bool saveHessian(const std::string& fileName) const override;

  void multiplyHessian(double* dest, const double* src) const override {
    Hpp_->multiplySymmetricUpperTriangle(dest, src);
  }

 protected:
  void resize(int* blockPoseIndices, int numPoseBlocks,
              int* blockLandmarkIndices, int numLandmarkBlocks, int totalDim);

  void deallocate();

  std::unique_ptr<SparseBlockMatrix<PoseMatrixType>> Hpp_;
  std::unique_ptr<SparseBlockMatrix<LandmarkMatrixType>> Hll_;
  std::unique_ptr<SparseBlockMatrix<PoseLandmarkMatrixType>> Hpl_;

  std::unique_ptr<SparseBlockMatrix<PoseMatrixType>> Hschur_;
  std::unique_ptr<SparseBlockMatrixDiagonal<LandmarkMatrixType>> DInvSchur_;

  std::unique_ptr<SparseBlockMatrixCCS<PoseLandmarkMatrixType>> HplCCS_;
  std::unique_ptr<SparseBlockMatrixCCS<PoseMatrixType>> HschurTransposedCCS_;

  std::unique_ptr<LinearSolverType> linearSolver_;

  std::vector<PoseVectorType> diagonalBackupPose_;
  std::vector<LandmarkVectorType> diagonalBackupLandmark_;

#ifdef G2O_OPENMP
  std::vector<OpenMPMutex> coefficientsMutex_;
#endif

  std::unique_ptr<double[], AlignedDeleter<double>> coefficients_;
  std::unique_ptr<double[], AlignedDeleter<double>> bschur_;

  bool doSchur_ = true;
  int numPoses_ = 0;
  int numLandmarks_ = 0;
  int sizePoses_ = 0;
  int sizeLandmarks_ = 0;
};

template <typename Traits>
BlockSolver<Traits>::BlockSolver(std::unique_ptr<LinearSolverType> linearSolver)
    : BlockSolverBase(), linearSolver_(std::move(linearSolver)) {}

template <typename Traits>
void BlockSolver<Traits>::resize(int* blockPoseIndices, int numPoseBlocks,
                                 int* blockLandmarkIndices,
                                 int numLandmarkBlocks, int totalDim) {
  deallocate();

  resizeVector(totalDim);

  if (doSchur_) {
    // the following two are only used in schur
    assert(sizePoses_ > 0 && "allocating with wrong size");
    coefficients_.reset(allocate_aligned<double>(totalDim));
    bschur_.reset(allocate_aligned<double>(sizePoses_));
  }

  Hpp_ = std::make_unique<PoseHessianType>(blockPoseIndices, blockPoseIndices,
                                           numPoseBlocks, numPoseBlocks);
  if (doSchur_) {
    Hschur_ = std::make_unique<PoseHessianType>(
        blockPoseIndices, blockPoseIndices, numPoseBlocks, numPoseBlocks);
    Hll_ = std::make_unique<LandmarkHessianType>(
        blockLandmarkIndices, blockLandmarkIndices, numLandmarkBlocks,
        numLandmarkBlocks);
    DInvSchur_ =
        std::make_unique<SparseBlockMatrixDiagonal<LandmarkMatrixType>>(
            Hll_->colBlockIndices());
    Hpl_ = std::make_unique<PoseLandmarkHessianType>(
        blockPoseIndices, blockLandmarkIndices, numPoseBlocks,
        numLandmarkBlocks);
    HplCCS_ = std::make_unique<SparseBlockMatrixCCS<PoseLandmarkMatrixType>>(
        Hpl_->rowBlockIndices(), Hpl_->colBlockIndices());
    HschurTransposedCCS_ =
        std::make_unique<SparseBlockMatrixCCS<PoseMatrixType>>(
            Hschur_->colBlockIndices(), Hschur_->rowBlockIndices());
#ifdef G2O_OPENMP
    coefficientsMutex_.resize(numPoseBlocks);
#endif
  }
}

template <typename Traits>
void BlockSolver<Traits>::deallocate() {
  Hpp_.reset();
  Hll_.reset();
  Hpl_.reset();
  Hschur_.reset();
  DInvSchur_.reset();
  coefficients_.reset();
  bschur_.reset();

  HplCCS_.reset();
  HschurTransposedCCS_.reset();
}

template <typename Traits>
BlockSolver<Traits>::~BlockSolver() = default;

template <typename Traits>
bool BlockSolver<Traits>::buildStructure(bool zeroBlocks) {
  assert(optimizer_);

  size_t sparseDim = 0;
  numPoses_ = 0;
  numLandmarks_ = 0;
  sizePoses_ = 0;
  sizeLandmarks_ = 0;
  int* blockPoseIndices = new int[optimizer_->indexMapping().size()];
  int* blockLandmarkIndices = new int[optimizer_->indexMapping().size()];

  for (auto* v : optimizer_->indexMapping()) {
    int dim = v->dimension();
    if (!v->marginalized()) {
      v->setColInHessian(sizePoses_);
      sizePoses_ += dim;
      blockPoseIndices[numPoses_] = sizePoses_;
      ++numPoses_;
    } else {
      v->setColInHessian(sizeLandmarks_);
      sizeLandmarks_ += dim;
      blockLandmarkIndices[numLandmarks_] = sizeLandmarks_;
      ++numLandmarks_;
    }
    sparseDim += dim;
  }
  resize(blockPoseIndices, numPoses_, blockLandmarkIndices, numLandmarks_,
         sparseDim);
  delete[] blockLandmarkIndices;
  delete[] blockPoseIndices;

  // allocate the diagonal on Hpp and Hll
  int poseIdx = 0;
  int landmarkIdx = 0;
  for (auto* v : optimizer_->indexMapping()) {
    if (!v->marginalized()) {
      // assert(poseIdx == v->hessianIndex());
      PoseMatrixType* m = Hpp_->block(poseIdx, poseIdx, true);
      if (zeroBlocks) m->setZero();
      v->mapHessianMemory(m->data());
      ++poseIdx;
    } else {
      LandmarkMatrixType* m = Hll_->block(landmarkIdx, landmarkIdx, true);
      if (zeroBlocks) m->setZero();
      v->mapHessianMemory(m->data());
      ++landmarkIdx;
    }
  }
  assert(poseIdx == numPoses_ && landmarkIdx == numLandmarks_);

  // temporary structures for building the pattern of the Schur complement
  SparseBlockMatrixHashMap<PoseMatrixType>* schurMatrixLookup = nullptr;
  if (doSchur_) {
    schurMatrixLookup = new SparseBlockMatrixHashMap<PoseMatrixType>(
        Hschur_->rowBlockIndices(), Hschur_->colBlockIndices());
    schurMatrixLookup->blockCols().resize(Hschur_->blockCols().size());
  }

  // here we assume that the landmark indices start after the pose ones
  // create the structure in Hpp, Hll and in Hpl
  for (const auto& e : optimizer_->activeEdges()) {
    for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
      auto v1 =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertex(viIdx));
      int ind1 = v1->hessianIndex();
      if (ind1 == -1) continue;
      int indexV1Bak = ind1;
      for (size_t vjIdx = viIdx + 1; vjIdx < e->vertices().size(); ++vjIdx) {
        auto v2 = std::static_pointer_cast<OptimizableGraph::Vertex>(
            e->vertex(vjIdx));
        int ind2 = v2->hessianIndex();
        if (ind2 == -1) continue;
        ind1 = indexV1Bak;
        bool transposedBlock = ind1 > ind2;
        if (transposedBlock) {  // make sure, we allocate the upper triangle
                                // block
          std::swap(ind1, ind2);
        }
        if (!v1->marginalized() && !v2->marginalized()) {
          PoseMatrixType* m = Hpp_->block(ind1, ind2, true);
          if (zeroBlocks) m->setZero();
          e->mapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);
          if (Hschur_) {  // assume this is only needed in case we solve with
                          // the schur complement
            schurMatrixLookup->addBlock(ind1, ind2);
          }
        } else if (v1->marginalized() && v2->marginalized()) {
          // RAINER hmm.... should we ever reach this here????
          LandmarkMatrixType* m =
              Hll_->block(ind1 - numPoses_, ind2 - numPoses_, true);
          if (zeroBlocks) m->setZero();
          e->mapHessianMemory(m->data(), viIdx, vjIdx, false);
        } else {
          if (v1->marginalized()) {
            PoseLandmarkMatrixType* m = Hpl_->block(
                v2->hessianIndex(), v1->hessianIndex() - numPoses_, true);
            if (zeroBlocks) m->setZero();
            e->mapHessianMemory(
                m->data(), viIdx, vjIdx,
                true);  // transpose the block before writing to it
          } else {
            PoseLandmarkMatrixType* m = Hpl_->block(
                v1->hessianIndex(), v2->hessianIndex() - numPoses_, true);
            if (zeroBlocks) m->setZero();
            e->mapHessianMemory(m->data(), viIdx, vjIdx,
                                false);  // directly the block
          }
        }
      }
    }
  }

  if (!doSchur_) {
    delete schurMatrixLookup;
    return true;
  }

  DInvSchur_->diagonal().resize(landmarkIdx);
  Hpl_->fillSparseBlockMatrixCCS(*HplCCS_);

  for (OptimizableGraph::Vertex* v : optimizer_->indexMapping()) {
    if (v->marginalized()) {
      const HyperGraph::EdgeSetWeak& vedges = v->edges();
      for (auto it1 = vedges.begin(); it1 != vedges.end(); ++it1) {
        auto e1 = it1->lock();
        for (size_t i = 0; i < e1->vertices().size(); ++i) {
          auto v1 =
              std::static_pointer_cast<OptimizableGraph::Vertex>(e1->vertex(i));
          if (v1->hessianIndex() == -1 || v1.get() == v) continue;
          for (const auto& vedge : vedges) {
            auto e2 = vedge.lock();
            for (size_t j = 0; j < e2->vertices().size(); ++j) {
              auto v2 = std::static_pointer_cast<OptimizableGraph::Vertex>(
                  e2->vertex(j));
              if (v2->hessianIndex() == -1 || v2.get() == v) continue;
              int i1 = v1->hessianIndex();
              int i2 = v2->hessianIndex();
              if (i1 <= i2) {
                schurMatrixLookup->addBlock(i1, i2);
              }
            }
          }
        }
      }
    }
  }

  Hschur_->takePatternFromHash(*schurMatrixLookup);
  delete schurMatrixLookup;
  Hschur_->fillSparseBlockMatrixCCSTransposed(*HschurTransposedCCS_);

  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::updateStructure(
    const HyperGraph::VertexContainer& vset, const HyperGraph::EdgeSet& edges) {
  for (const auto& vit : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(vit.get());
    int dim = v->dimension();
    if (!v->marginalized()) {
      v->setColInHessian(sizePoses_);
      sizePoses_ += dim;
      Hpp_->rowBlockIndices().push_back(sizePoses_);
      Hpp_->colBlockIndices().push_back(sizePoses_);
      Hpp_->blockCols().push_back(
          typename SparseBlockMatrix<PoseMatrixType>::IntBlockMap());
      ++numPoses_;
      int ind = v->hessianIndex();
      PoseMatrixType* m = Hpp_->block(ind, ind, true);
      v->mapHessianMemory(m->data());
    } else {
      G2O_ERROR("updateStructure(): Schur not supported");
      abort();
    }
  }
  resizeVector(sizePoses_ + sizeLandmarks_);

  for (const auto& e : edges) {
    for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
      auto v1 =
          std::static_pointer_cast<OptimizableGraph::Vertex>(e->vertex(viIdx));
      int ind1 = v1->hessianIndex();
      int indexV1Bak = ind1;
      if (ind1 == -1) continue;
      for (size_t vjIdx = viIdx + 1; vjIdx < e->vertices().size(); ++vjIdx) {
        auto v2 = std::static_pointer_cast<OptimizableGraph::Vertex>(
            e->vertex(vjIdx));
        int ind2 = v2->hessianIndex();
        if (ind2 == -1) continue;
        ind1 = indexV1Bak;
        bool transposedBlock = ind1 > ind2;
        if (transposedBlock)  // make sure, we allocate the upper triangular
                              // block
          std::swap(ind1, ind2);

        if (!v1->marginalized() && !v2->marginalized()) {
          PoseMatrixType* m = Hpp_->block(ind1, ind2, true);
          auto* ee = static_cast<OptimizableGraph::Edge*>(e.get());
          ee->mapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);
        } else {
          G2O_ERROR("{}: not supported", __PRETTY_FUNCTION__);
        }
      }
    }
  }

  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::solve() {
  if (!doSchur_) {
    double t = get_monotonic_time();
    bool ok = linearSolver_->solve(*Hpp_, x_, b_);
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeLinearSolver = get_monotonic_time() - t;
      globalStats->hessianDimension = globalStats->hessianPoseDimension =
          Hpp_->cols();
    }
    return ok;
  }

  // schur thing

  // backup the coefficient matrix
  double t = get_monotonic_time();

  // _Hschur = _Hpp, but keeping the pattern of _Hschur
  Hschur_->clear();
  Hpp_->add(*Hschur_);

  //_DInvSchur->clear();
  memset(coefficients_.get(), 0, sizePoses_ * sizeof(double));
#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) schedule(dynamic, 10)
#endif
  for (int landmarkIndex = 0;
       landmarkIndex < static_cast<int>(Hll_->blockCols().size());
       ++landmarkIndex) {
    const typename SparseBlockMatrix<LandmarkMatrixType>::IntBlockMap&
        marginalizeColumn = Hll_->blockCols()[landmarkIndex];
    assert(marginalizeColumn.size() == 1 &&
           "more than one block in _Hll column");

    // calculate inverse block for the landmark
    const LandmarkMatrixType* D = marginalizeColumn.begin()->second;
    assert(D && D->rows() == D->cols() && "Error in landmark matrix");
    LandmarkMatrixType& Dinv = DInvSchur_->diagonal()[landmarkIndex];
    Dinv = D->inverse();

    LandmarkVectorType db(D->rows());
    for (int j = 0; j < D->rows(); ++j) {
      db[j] = b_[Hll_->rowBaseOfBlock(landmarkIndex) + sizePoses_ + j];
    }
    db = Dinv * db;

    assert((size_t)landmarkIndex < HplCCS_->blockCols().size() &&
           "Index out of bounds");
    const typename SparseBlockMatrixCCS<PoseLandmarkMatrixType>::SparseColumn&
        landmarkColumn = HplCCS_->blockCols()[landmarkIndex];

    for (auto it_outer = landmarkColumn.begin();
         it_outer != landmarkColumn.end(); ++it_outer) {
      int i1 = it_outer->row;

      const PoseLandmarkMatrixType* Bi = it_outer->block;
      assert(Bi);

      PoseLandmarkMatrixType BDinv = (*Bi) * (Dinv);
      assert(HplCCS_->rowBaseOfBlock(i1) < sizePoses_ && "Index out of bounds");
      typename PoseVectorType::MapType Bb(
          &coefficients_[HplCCS_->rowBaseOfBlock(i1)], Bi->rows());
#ifdef G2O_OPENMP
      ScopedOpenMPMutex mutexLock(&coefficientsMutex_[i1]);
#endif
      Bb.noalias() += (*Bi) * db;

      assert(i1 >= 0 &&
             i1 < static_cast<int>(HschurTransposedCCS_->blockCols().size()) &&
             "Index out of bounds");
      auto targetColumnIt = HschurTransposedCCS_->blockCols()[i1].begin();

      typename SparseBlockMatrixCCS<PoseLandmarkMatrixType>::RowBlock aux(
          i1, nullptr);
      auto it_inner =
          lower_bound(landmarkColumn.begin(), landmarkColumn.end(), aux);
      for (; it_inner != landmarkColumn.end(); ++it_inner) {
        int i2 = it_inner->row;
        const PoseLandmarkMatrixType* Bj = it_inner->block;
        assert(Bj);
        while (targetColumnIt->row < i2 /*&& targetColumnIt != HschurTransposedCCS_->blockCols()[i1].end()*/)
          ++targetColumnIt;
        assert(targetColumnIt != HschurTransposedCCS_->blockCols()[i1].end() &&
               targetColumnIt->row == i2 &&
               "invalid iterator, something wrong with the matrix structure");
        PoseMatrixType* Hi1i2 = targetColumnIt->block;  //_Hschur->block(i1,i2);
        assert(Hi1i2);
        (*Hi1i2).noalias() -= BDinv * Bj->transpose();
      }
    }
  }

  // _bschur = _b for calling solver, and not touching _b
  memcpy(bschur_.get(), b_, sizePoses_ * sizeof(double));
  for (int i = 0; i < sizePoses_; ++i) {
    bschur_[i] -= coefficients_[i];
  }

  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeSchurComplement = get_monotonic_time() - t;
  }

  t = get_monotonic_time();
  bool solvedPoses = linearSolver_->solve(*Hschur_, x_, bschur_.get());
  if (globalStats) {
    globalStats->timeLinearSolver = get_monotonic_time() - t;
    globalStats->hessianPoseDimension = Hpp_->cols();
    globalStats->hessianLandmarkDimension = Hll_->cols();
    globalStats->hessianDimension = globalStats->hessianPoseDimension +
                                    globalStats->hessianLandmarkDimension;
  }

  if (!solvedPoses) return false;

  // _x contains the solution for the poses, now applying it to the landmarks to
  // get the new part of the solution;
  double* xp = x_;
  double* cp = coefficients_.get();

  double* xl = x_ + sizePoses_;
  double* cl = coefficients_.get() + sizePoses_;
  double* bl = b_ + sizePoses_;

  // cp = -xp
  for (int i = 0; i < sizePoses_; ++i) cp[i] = -xp[i];

  // cl = bl
  memcpy(cl, bl, sizeLandmarks_ * sizeof(double));

  // cl = bl - Bt * xp
  // Bt->multiply(cl, cp);
  HplCCS_->rightMultiply(cl, cp);

  // xl = Dinv * cl
  memset(xl, 0, sizeLandmarks_ * sizeof(double));
  DInvSchur_->multiply(xl, cl);
  //_DInvSchur->rightMultiply(xl,cl);

  return true;
}

template <typename Traits>
bool BlockSolver<Traits>::computeMarginals(
    SparseBlockMatrix<MatrixX>& spinv,
    const std::vector<std::pair<int, int>>& blockIndices) {
  double t = get_monotonic_time();
  bool ok = linearSolver_->solvePattern(spinv, blockIndices, *Hpp_);
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeMarginals = get_monotonic_time() - t;
  }
  return ok;
}

template <typename Traits>
bool BlockSolver<Traits>::buildSystem() {
  // clear b vector
#ifdef G2O_OPENMP
#pragma omp parallel for default( \
        shared) if (optimizer_->indexMapping().size() > 1000)
#endif
  for (auto* v : optimizer_->indexMapping()) {
    assert(v);
    v->clearQuadraticForm();
  }
  Hpp_->clear();
  if (doSchur_) {
    Hll_->clear();
    Hpl_->clear();
  }

  // resetting the terms for the pairwise constraints
  // built up the current system by storing the Hessian blocks in the edges and
  // vertices
#ifndef G2O_OPENMP
  // no threading, we do not need to copy the workspace
  JacobianWorkspace& jacobianWorkspace = optimizer_->jacobianWorkspace();
#else
  // if running with threads need to produce copies of the workspace for each
  // thread
  JacobianWorkspace jacobianWorkspace = optimizer_->jacobianWorkspace();
#pragma omp parallel for default(shared) firstprivate( \
        jacobianWorkspace) if (optimizer_->activeEdges().size() > 100)
#endif
  for (const auto& e : optimizer_->activeEdges()) {
    e->linearizeOplus(
        jacobianWorkspace);  // jacobian of the nodes' oplus (manifold)
    e->constructQuadraticForm();
#ifndef NDEBUG
    for (size_t i = 0; i < e->vertices().size(); ++i) {
      auto v = std::static_pointer_cast<const OptimizableGraph::Vertex>(
          e->vertex(i));
      if (!v->fixed()) {
        bool hasANan = arrayHasNaN(jacobianWorkspace.workspaceForVertex(i),
                                   e->dimension() * v->dimension());
        if (hasANan) {
          G2O_WARN(
              "buildSystem(): NaN within Jacobian for edge {} for vertex {}",
              static_cast<void*>(e.get()), i);
          break;
        }
      }
    }
#endif
  }

  // flush the current system in a sparse block matrix
#ifdef G2O_OPENMP
#pragma omp parallel for default( \
        shared) if (optimizer_->indexMapping().size() > 1000)
#endif
  for (auto* v : optimizer_->indexMapping()) {
    int iBase = v->colInHessian();
    if (v->marginalized()) iBase += sizePoses_;
    v->copyB(b_ + iBase);
  }

  return false;
}

template <typename Traits>
bool BlockSolver<Traits>::setLambda(double lambda, bool backup) {
  if (backup) {
    diagonalBackupPose_.resize(numPoses_);
    diagonalBackupLandmark_.resize(numLandmarks_);
  }
#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (numPoses_ > 100)
#endif
  for (int i = 0; i < numPoses_; ++i) {
    PoseMatrixType* b = Hpp_->block(i, i);
    if (backup) diagonalBackupPose_[i] = b->diagonal();
    b->diagonal().array() += lambda;
  }
#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (numLandmarks_ > 100)
#endif
  for (int i = 0; i < numLandmarks_; ++i) {
    LandmarkMatrixType* b = Hll_->block(i, i);
    if (backup) diagonalBackupLandmark_[i] = b->diagonal();
    b->diagonal().array() += lambda;
  }
  return true;
}

template <typename Traits>
void BlockSolver<Traits>::restoreDiagonal() {
  assert((int)diagonalBackupPose_.size() == numPoses_ &&
         "Mismatch in dimensions");
  assert((int)diagonalBackupLandmark_.size() == numLandmarks_ &&
         "Mismatch in dimensions");
  for (int i = 0; i < numPoses_; ++i) {
    PoseMatrixType* b = Hpp_->block(i, i);
    b->diagonal() = diagonalBackupPose_[i];
  }
  for (int i = 0; i < numLandmarks_; ++i) {
    LandmarkMatrixType* b = Hll_->block(i, i);
    b->diagonal() = diagonalBackupLandmark_[i];
  }
}

template <typename Traits>
bool BlockSolver<Traits>::init(SparseOptimizer* optimizer, bool online) {
  optimizer_ = optimizer;
  if (!online) {
    if (Hpp_) Hpp_->clear();
    if (Hpl_) Hpl_->clear();
    if (Hll_) Hll_->clear();
  }
  linearSolver_->init();
  return true;
}

template <typename Traits>
void BlockSolver<Traits>::setWriteDebug(bool writeDebug) {
  linearSolver_->setWriteDebug(writeDebug);
}

template <typename Traits>
bool BlockSolver<Traits>::saveHessian(const std::string& fileName) const {
  return Hpp_->writeOctave(fileName.c_str(), true);
}

template <int P, int L>
using BlockSolverPL = BlockSolver<BlockSolverTraits<P, L>>;

// variable size solver
using BlockSolverX = BlockSolverPL<Eigen::Dynamic, Eigen::Dynamic>;

// solver for BA/3D SLAM
using BlockSolver_6_3 = BlockSolverPL<6, 3>;

// solver fo BA with scale
using BlockSolver_7_3 = BlockSolverPL<7, 3>;

// 2Dof landmarks 3Dof poses
using BlockSolver_3_2 = BlockSolverPL<3, 2>;

}  // namespace g2o

#endif
