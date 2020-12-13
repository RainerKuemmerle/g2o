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

#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "gtest/gtest.h"
#include "sparse_system_helper.h"

TEST(LinearSolver, Solve) {
  g2o::SparseBlockMatrixX sparse_matrix = g2o::internal::createTestMatrix();
  g2o::VectorX x_vector = g2o::internal::createTestVectorX();
  g2o::VectorX b_vector = g2o::internal::createTestVectorB();

  auto linearSolver =
      g2o::make_unique<g2o::LinearSolverEigen<g2o::SparseBlockMatrixX::SparseMatrixBlock>>();
  linearSolver->setBlockOrdering(true /* block ordering */);

  g2o::VectorX solver_solution;
  solver_solution.resize(b_vector.size());
  solver_solution.fill(0.);
  linearSolver->solve(sparse_matrix, solver_solution.data(), b_vector.data());

  ASSERT_TRUE(solver_solution.isApprox(x_vector, 1e-6));
}
