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

#include "g2o/core/sparse_block_matrix.h"

#include <iostream>

#include "gtest/gtest.h"

using namespace std;
using namespace g2o;

using SparseBlockMatrixX = SparseBlockMatrix<MatrixX>;

std::ostream& operator<<(std::ostream& os,
                         const SparseBlockMatrixX::SparseMatrixBlock& m) {
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) os << m(i, j) << " ";
    os << endl;
  }
  return os;
}

TEST(General, SparseBlockMatrix) {
  auto fill_block = [](MatrixX& b) {
    for (int i = 0; i < b.rows(); ++i)
      for (int j = 0; j < b.cols(); ++j) {
        b(i, j) = i * b.cols() + j;
      }
  };

  int rcol[] = {3, 6, 8, 12};
  int ccol[] = {2, 4, 13};
  SparseBlockMatrixX* M = new SparseBlockMatrixX(rcol, ccol, 4, 3);

  SparseBlockMatrixX::SparseMatrixBlock* b = M->block(0, 0, true);
  EXPECT_EQ(3, b->rows());
  EXPECT_EQ(2, b->cols());
  fill_block(*b);

  b = M->block(0, 2, true);
  EXPECT_EQ(3, b->rows());
  EXPECT_EQ(9, b->cols());
  fill_block(*b);

  b = M->block(3, 2, true);
  EXPECT_EQ(4, b->rows());
  EXPECT_EQ(9, b->cols());
  fill_block(*b);

  auto Ms = M->added();
  bool addResult = M->add(*Ms);
  EXPECT_TRUE(addResult);
  EXPECT_EQ(Ms->rows(), M->rows());
  EXPECT_EQ(Ms->cols(), M->cols());

  auto Mt = M->transposed<MatrixX>();
  EXPECT_EQ(M->rows(), Mt->cols());
  EXPECT_EQ(M->cols(), Mt->rows());
  // cerr << *Mt << endl;

  SparseBlockMatrixX* Mp = nullptr;
  M->multiply(Mp, Mt.get());
  ASSERT_NE(Mp, nullptr);

  int iperm[] = {3, 2, 1, 0};
  SparseBlockMatrixX* PMp = nullptr;
  bool symPermResult = Mp->symmPermutation(PMp, iperm, false);
  ASSERT_TRUE(symPermResult);
  // cerr << *PMp << endl;

  PMp->clear(true);
  Mp->block(3, 0)->fill(0.);
  symPermResult = Mp->symmPermutation(PMp, iperm, true);
  ASSERT_TRUE(symPermResult);
  // cerr << *PMp << endl;
}
