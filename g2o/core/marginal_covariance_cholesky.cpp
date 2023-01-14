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

#include "marginal_covariance_cholesky.h"

#include <algorithm>
#include <cassert>

#include "g2o/core/eigen_types.h"

namespace g2o {

struct MatrixElem {
  int r, c;
  MatrixElem(int r_, int c_) : r(r_), c(c_) {}
  bool operator<(const MatrixElem& other) const {
    return c > other.c || (c == other.c && r > other.r);
  }
};

void MarginalCovarianceCholesky::setCholeskyFactor(int n, int* Lp, int* Li,
                                                   number_t* Lx, int* permInv) {
  n_ = n;
  Ap_ = Lp;
  Ai_ = Li;
  Ax_ = Lx;
  perm_ = permInv;

  // pre-compute reciprocal values of the diagonal of L
  diag_.resize(n);
  for (int r = 0; r < n; ++r) {
    const int& sc = Ap_[r];  // L is lower triangular, thus the first elem in
                             // the column is the diagonal entry
    assert(r == Ai_[sc] && "Error in CCS storage of L");
    diag_[r] = 1.0 / Ax_[sc];
  }
}

number_t MarginalCovarianceCholesky::computeEntry(int r, int c)  // NOLINT
{
  assert(r <= c);
  const int idx = computeIndex(r, c);

  const LookupMap::const_iterator foundIt = map_.find(idx);
  if (foundIt != map_.end()) {
    return foundIt->second;
  }

  // compute the summation over column r
  number_t s = 0.;
  const int& sc = Ap_[r];
  const int& ec = Ap_[r + 1];
  // sum over row r while skipping the element on the diagonal
  for (int j = sc + 1; j < ec; ++j) {
    const int& rr = Ai_[j];
    const number_t val =
        rr < c ? computeEntry(rr, c) : computeEntry(c, rr);  // NOLINT
    s += val * Ax_[j];
  }

  number_t result;
  if (r == c) {
    const number_t& diagElem = diag_[r];
    result = diagElem * (diagElem - s);
  } else {
    result = -s * diag_[r];
  }
  map_[idx] = result;
  return result;
}

void MarginalCovarianceCholesky::computeCovariance(
    number_t** covBlocks, const std::vector<int>& blockIndices) {
  map_.clear();
  int base = 0;
  std::vector<MatrixElem> elemsToCompute;
  for (const int nbase : blockIndices) {
    const int vdim = nbase - base;
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = perm_ ? perm_[rr + base] : rr + base;  // apply permutation
        int c = perm_ ? perm_[cc + base] : cc + base;
        if (r > c)  // make sure it's still upper triangular after applying the
                    // permutation
          std::swap(r, c);
        elemsToCompute.emplace_back(r, c);
      }
    base = nbase;
  }

  // sort the elems to reduce the recursive calls
  std::sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (auto& me : elemsToCompute) {
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance for the vertices, by writing to the blocks
  // memory
  base = 0;
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    const int nbase = blockIndices[i];
    const int vdim = nbase - base;
    number_t* cov = covBlocks[i];
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = perm_ ? perm_[rr + base] : rr + base;  // apply permutation
        int c = perm_ ? perm_[cc + base] : cc + base;
        if (r > c)  // upper triangle
          std::swap(r, c);
        const int idx = computeIndex(r, c);
        const LookupMap::const_iterator foundIt = map_.find(idx);
        assert(foundIt != map_.end());
        cov[rr * vdim + cc] = foundIt->second;
        if (rr != cc) cov[cc * vdim + rr] = foundIt->second;
      }
    base = nbase;
  }
}

void MarginalCovarianceCholesky::computeCovariance(
    SparseBlockMatrix<MatrixX>& spinv, const std::vector<int>& rowBlockIndices,
    const std::vector<std::pair<int, int> >& blockIndices) {
  // allocate the sparse
  spinv = SparseBlockMatrix<MatrixX>(
      rowBlockIndices.data(), rowBlockIndices.data(), rowBlockIndices.size(),
      rowBlockIndices.size(), true);
  map_.clear();
  std::vector<MatrixElem> elemsToCompute;
  for (const auto& blockIndex : blockIndices) {
    const int blockRow = blockIndex.first;
    const int blockCol = blockIndex.second;
    assert(blockRow >= 0);
    assert(blockRow < (int)rowBlockIndices.size());
    assert(blockCol >= 0);
    assert(blockCol < (int)rowBlockIndices.size());

    const int rowBase = spinv.rowBaseOfBlock(blockRow);
    const int colBase = spinv.colBaseOfBlock(blockCol);

    MatrixX* block = spinv.block(blockRow, blockCol, true);
    assert(block);
    for (int iRow = 0; iRow < block->rows(); ++iRow)
      for (int iCol = 0; iCol < block->cols(); ++iCol) {
        const int rr = rowBase + iRow;
        const int cc = colBase + iCol;
        int r = perm_ ? perm_[rr] : rr;  // apply permutation
        int c = perm_ ? perm_[cc] : cc;
        if (r > c) std::swap(r, c);
        elemsToCompute.emplace_back(r, c);
      }
  }

  // sort the elems to reduce the number of recursive calls
  sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (auto& me : elemsToCompute) {
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance
  for (const auto& blockIndex : blockIndices) {
    const int blockRow = blockIndex.first;
    const int blockCol = blockIndex.second;
    const int rowBase = spinv.rowBaseOfBlock(blockRow);
    const int colBase = spinv.colBaseOfBlock(blockCol);

    MatrixX* block = spinv.block(blockRow, blockCol);
    assert(block);
    for (int iRow = 0; iRow < block->rows(); ++iRow)
      for (int iCol = 0; iCol < block->cols(); ++iCol) {
        const int rr = rowBase + iRow;
        const int cc = colBase + iCol;
        int r = perm_ ? perm_[rr] : rr;  // apply permutation
        int c = perm_ ? perm_[cc] : cc;
        if (r > c) std::swap(r, c);
        const int idx = computeIndex(r, c);
        const LookupMap::const_iterator foundIt = map_.find(idx);
        assert(foundIt != map_.end());
        (*block)(iRow, iCol) = foundIt->second;
      }
  }
}

}  // namespace g2o
