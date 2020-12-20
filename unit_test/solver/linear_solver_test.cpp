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

// clang-format off
#include "g2o/config.h"
// clang-format on

#include "g2o/solvers/eigen/linear_solver_eigen.h"
#ifdef G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif
#ifdef G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
#include "gtest/gtest.h"
#include "sparse_system_helper.h"

struct BlockOrdering {
  static constexpr bool blockOrdering = true;
};

struct NoBlockOrdering {
  static constexpr bool blockOrdering = false;
};

/**
 * Type parameterized class for a fixture to setup a linear solver along with some data of a linear
 * system to be solved.
 */
template <typename T>
class LS : public testing::Test {
 public:
  using LinearSolverType = typename T::first_type;
  using OrderingType = typename T::second_type;

  LS() : linearsolver(g2o::make_unique<LinearSolverType>()) {}

 protected:
  std::unique_ptr<LinearSolverType> linearsolver;
  g2o::SparseBlockMatrixX sparse_matrix = g2o::internal::createTestMatrix();
  g2o::MatrixX matrix_inverse = g2o::internal::createTestMatrixInverse();
  g2o::VectorX x_vector = g2o::internal::createTestVectorX();
  g2o::VectorX b_vector = g2o::internal::createTestVectorB();
};
TYPED_TEST_SUITE_P(LS);

TYPED_TEST_P(LS, Solve) {
  this->linearsolver->setBlockOrdering(TypeParam::second_type::blockOrdering);

  g2o::VectorX solver_solution;
  for (int solve_iter = 0; solve_iter < 2; ++solve_iter) {
    solver_solution.setZero(this->b_vector.size());
    this->linearsolver->solve(this->sparse_matrix, solver_solution.data(), this->b_vector.data());

    ASSERT_TRUE(solver_solution.isApprox(this->x_vector, 1e-6))
        << "Solution differs on iteration " << solve_iter;
  }
}

TYPED_TEST_P(LS, SolvePattern) {
  this->linearsolver->setBlockOrdering(TypeParam::second_type::blockOrdering);

  g2o::SparseBlockMatrixX spinv;
  std::vector<std::pair<int, int> > blockIndices;
  for (int i = 0; i < static_cast<int>(this->sparse_matrix.rowBlockIndices().size()); ++i)
    blockIndices.emplace_back(std::make_pair(i, i));

  bool state = this->linearsolver->solvePattern(spinv, blockIndices, this->sparse_matrix);
  ASSERT_TRUE(!state || spinv.rowBlockIndices().size() == blockIndices.size());
  if (!state) {  // solver does not implement solving for a pattern return in this case
    std::cerr << "Solver does not support solvePattern()" << std::endl;
    SUCCEED();
    return;
  }

  for (const auto& idx : blockIndices) {
    int rr = spinv.rowBaseOfBlock(idx.first);
    int cc = spinv.colBaseOfBlock(idx.second);
    int numRows = spinv.rowsOfBlock(idx.first);
    int numCols = spinv.colsOfBlock(idx.second);

    g2o::MatrixX expected = this->matrix_inverse.block(rr, cc, numRows, numCols);
    g2o::MatrixX actual = *spinv.block(idx.first, idx.second);

    EXPECT_TRUE(actual.isApprox(expected, 1e-6))
        << "block " << idx.first << " " << idx.second << " differs";
  }
}

TYPED_TEST_P(LS, SolveBlocks) {
  this->linearsolver->setBlockOrdering(TypeParam::second_type::blockOrdering);

  number_t** blocks = nullptr;
  bool state = this->linearsolver->solveBlocks(blocks, this->sparse_matrix);
  ASSERT_TRUE(!state || blocks != nullptr);
  if (!state) {  // solver does not implement solving for a pattern return in this case
    std::cerr << "Solver does not support solveBlocks()" << std::endl;
    SUCCEED();
    return;
  }

  for (size_t i = 0; i < this->sparse_matrix.rowBlockIndices().size(); ++i) {
    int rr = this->sparse_matrix.rowBaseOfBlock(i);
    int cc = this->sparse_matrix.colBaseOfBlock(i);
    int numRows = this->sparse_matrix.rowsOfBlock(i);
    int numCols = this->sparse_matrix.colsOfBlock(i);

    g2o::MatrixX expected = this->matrix_inverse.block(rr, cc, numRows, numCols);
    g2o::MatrixX::MapType actual(blocks[i], numRows, numCols);

    EXPECT_TRUE(actual.isApprox(expected, 1e-6)) << "block " << i << " " << i << " differs";
  }
  TypeParam::first_type::deallocateBlocks(this->sparse_matrix, blocks);
}

// registering the test suite and all the types to be tested
REGISTER_TYPED_TEST_SUITE_P(LS, Solve, SolvePattern, SolveBlocks);

using LinearSolverTypes = ::testing::Types<
#ifdef G2O_HAVE_CSPARSE
    std::pair<g2o::LinearSolverCSparse<g2o::MatrixX>, NoBlockOrdering>,
    std::pair<g2o::LinearSolverCSparse<g2o::MatrixX>, BlockOrdering>,
#endif
#ifdef G2O_HAVE_CHOLMOD
    std::pair<g2o::LinearSolverCholmod<g2o::MatrixX>, NoBlockOrdering>,
    std::pair<g2o::LinearSolverCholmod<g2o::MatrixX>, BlockOrdering>,
#endif
    std::pair<g2o::LinearSolverEigen<g2o::MatrixX>, NoBlockOrdering>,
    std::pair<g2o::LinearSolverEigen<g2o::MatrixX>, BlockOrdering> >;
INSTANTIATE_TYPED_TEST_SUITE_P(LinearSolver, LS, LinearSolverTypes);
