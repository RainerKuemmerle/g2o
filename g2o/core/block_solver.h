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
#include "solver.h"
#include "linear_solver.h"
#include "sparse_block_matrix.h"
#include "sparse_block_matrix_diagonal.h"
#include "openmp_mutex.h"
#include "g2o/config.h"
#include "dynamic_aligned_buffer.hpp"

#include <memory>

namespace g2o {

  /**
   * \brief traits to summarize the properties of the fixed size optimization problem
   */
  template <int _PoseDim, int _LandmarkDim>
  struct BlockSolverTraits
  {
    static const int PoseDim = _PoseDim;
    static const int LandmarkDim = _LandmarkDim;
    using PoseMatrixType = Eigen::Matrix<number_t, PoseDim, PoseDim, Eigen::ColMajor>;
    using LandmarkMatrixType = Eigen::Matrix<number_t, LandmarkDim, LandmarkDim, Eigen::ColMajor>;
    using PoseLandmarkMatrixType = Eigen::Matrix<number_t, PoseDim, LandmarkDim, Eigen::ColMajor>;
    using PoseVectorType = Eigen::Matrix<number_t, PoseDim, 1, Eigen::ColMajor>;
    using LandmarkVectorType = Eigen::Matrix<number_t, LandmarkDim, 1, Eigen::ColMajor>;

    using PoseHessianType = SparseBlockMatrix<PoseMatrixType>;
    using LandmarkHessianType = SparseBlockMatrix<LandmarkMatrixType>;
    using PoseLandmarkHessianType = SparseBlockMatrix<PoseLandmarkMatrixType>;
    using LinearSolverType = LinearSolver<PoseMatrixType>;
  };

  /**
   * \brief traits to summarize the properties of the dynamic size optimization problem
   */
  template <>
  struct BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>
  {
    static const int PoseDim = Eigen::Dynamic;
    static const int LandmarkDim = Eigen::Dynamic;
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
  class BlockSolverBase : public Solver
  {
    public:
      ~BlockSolverBase() override = default;
      /**
       * compute dest = H * src
       */
      virtual void multiplyHessian(number_t* dest, const number_t* src) const = 0;
  };

  /**
   * \brief Implementation of a solver operating on the blocks of the Hessian
   */
  template <typename Traits>
  class BlockSolver: public BlockSolverBase
  {
    public:
      static const int PoseDim = Traits::PoseDim;
      static const int LandmarkDim = Traits::LandmarkDim;
      using PoseMatrixType = typename Traits::PoseMatrixType;
      using LandmarkMatrixType = typename Traits::LandmarkMatrixType;
      using PoseLandmarkMatrixType = typename Traits::PoseLandmarkMatrixType;
      using PoseVectorType = typename Traits::PoseVectorType;
      using LandmarkVectorType = typename Traits::LandmarkVectorType;

      using PoseHessianType = typename Traits::PoseHessianType;
      using LandmarkHessianType = typename Traits::LandmarkHessianType;
      using PoseLandmarkHessianType = typename Traits::PoseLandmarkHessianType;
      using LinearSolverType = typename Traits::LinearSolverType;

    public:

      /**
       * allocate a block solver ontop of the underlying linear solver.
       * NOTE: The BlockSolver assumes exclusive access to the linear solver and will therefore free the pointer
       * in its destructor.
       */
      explicit BlockSolver(std::unique_ptr<LinearSolverType> linearSolver);
      ~BlockSolver() override;

      bool init(SparseOptimizer* optmizer, bool online = false) override;
      bool buildStructure(bool zeroBlocks = false) override;
      bool updateStructure(const HyperGraph::VertexContainer& vset, const HyperGraph::EdgeSet& edges) override;
      bool buildSystem() override;
      bool solve() override;
      bool computeMarginals(SparseBlockMatrix<MatrixX>& spinv, const std::vector<std::pair<int, int> >& blockIndices) override;
      bool setLambda(number_t lambda, bool backup = false) override;
      void restoreDiagonal() override;
      bool supportsSchur() override {return true;}
      bool schur() override { return _doSchur;}
      void setSchur(bool s) override { _doSchur = s;}

      LinearSolver<PoseMatrixType>& linearSolver() const { return *_linearSolver;}

      void setWriteDebug(bool writeDebug) override;
      bool writeDebug() const override {return _linearSolver->writeDebug();}

      bool saveHessian(const std::string& fileName) const override;

      void multiplyHessian(number_t* dest, const number_t* src) const override { _Hpp->multiplySymmetricUpperTriangle(dest, src);}

    protected:
      void resize(int* blockPoseIndices, int numPoseBlocks,
          int* blockLandmarkIndices, int numLandmarkBlocks, int totalDim);

      void deallocate();

      std::unique_ptr<SparseBlockMatrix<PoseMatrixType>> _Hpp;
      std::unique_ptr<SparseBlockMatrix<LandmarkMatrixType>> _Hll;
      std::unique_ptr<SparseBlockMatrix<PoseLandmarkMatrixType>> _Hpl;

      std::unique_ptr<SparseBlockMatrix<PoseMatrixType>> _Hschur;
      std::unique_ptr<SparseBlockMatrixDiagonal<LandmarkMatrixType>> _DInvSchur;

      std::unique_ptr<SparseBlockMatrixCCS<PoseLandmarkMatrixType>> _HplCCS;
      std::unique_ptr<SparseBlockMatrixCCS<PoseMatrixType>> _HschurTransposedCCS;

      std::unique_ptr<LinearSolverType> _linearSolver;

      std::vector<PoseVectorType, Eigen::aligned_allocator<PoseVectorType> > _diagonalBackupPose;
      std::vector<LandmarkVectorType, Eigen::aligned_allocator<LandmarkVectorType> > _diagonalBackupLandmark;

#    ifdef G2O_OPENMP
      std::vector<OpenMPMutex> _coefficientsMutex;
#    endif

      bool _doSchur;

      std::unique_ptr<number_t[], aligned_deleter<number_t>> _coefficients;
      std::unique_ptr<number_t[], aligned_deleter<number_t>> _bschur;

      int _numPoses, _numLandmarks;
      int _sizePoses, _sizeLandmarks;
  };


  template<int p, int l>
  using BlockSolverPL = BlockSolver< BlockSolverTraits<p, l> >;

  //variable size solver
  using BlockSolverX = BlockSolverPL<Eigen::Dynamic, Eigen::Dynamic>;

  // solver for BA/3D SLAM
  using BlockSolver_6_3 = BlockSolverPL<6, 3>;

  // solver fo BA with scale
  using BlockSolver_7_3 = BlockSolverPL<7, 3>;

  // 2Dof landmarks 3Dof poses
  using BlockSolver_3_2 = BlockSolverPL<3, 2>;

} // end namespace

#include "block_solver.hpp"


#endif
