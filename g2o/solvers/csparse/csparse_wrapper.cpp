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

#include "csparse_wrapper.h"

#include <cassert>
#include <cstddef>
#include <cstring>
#include <memory>

#include "csparse_extension.h"
#include "csparse_helper.h"
#include "g2o/core/eigen_types.h"

namespace g2o {
namespace csparse {

/**
 * \brief Our C++ version of the csparse struct
 */
struct CSparseExt : public cs {
  CSparseExt() {
    nzmax = 0;
    m = 0;
    n = 0;
    p = 0;
    i = 0;
    x = 0;
    nz = -1;  // tag as CCS formatted matrix
  }
  CSparseExt(CSparseExt const&) = delete;
  CSparseExt& operator=(CSparseExt const&) = delete;
  ~CSparseExt() {
    delete[] p;
    delete[] i;
    delete[] x;
  }
  int columnsAllocated = 0;
};

/**
 * Pimpl-Idiom to hold the data interfacing with CSparse
 */
class CSparse::Impl {
 public:
  Impl() = default;
  ~Impl() {
    if (symbolicDecomposition) {
      cs_sfree(symbolicDecomposition);
      symbolicDecomposition = nullptr;
    }
    if (numericCholesky) {
      cs_nfree(numericCholesky);
      numericCholesky = nullptr;
    }
    delete[] csWorkspace;
    csWorkspace = nullptr;
    delete[] csIntWorkspace;
    csIntWorkspace = nullptr;
  }

  void prepareWorkspace() {
    // re-allocate the temporary workspace for cholesky
    if (csWorkspaceSize < ccsA.n) {
      csWorkspaceSize = csWorkspaceSize == 0 ? ccsA.n : 2 * ccsA.n;
      delete[] csWorkspace;
      csWorkspace = new number_t[csWorkspaceSize];
      delete[] csIntWorkspace;
      csIntWorkspace = new int[2 * csWorkspaceSize];
    }
  }

  css* symbolicDecomposition = nullptr;
  int csWorkspaceSize = 0;
  number_t* csWorkspace = nullptr;
  int* csIntWorkspace = nullptr;
  csn* numericCholesky = nullptr;
  CSparseExt ccsA;
};

CSparse::CSparse() : pImpl_(std::make_unique<Impl>()) {}

CSparse::~CSparse() = default;

void CSparse::freeFactor() {
  if (pImpl_->numericCholesky) {
    cs_nfree(pImpl_->numericCholesky);
    pImpl_->numericCholesky = nullptr;
  }
}

bool CSparse::hasFactor() const { return pImpl_->numericCholesky != nullptr; }

bool CSparse::amd(const SparseView& sparseView, VectorXI& result) {
  // prepare block structure for the CSparse call
  cs auxBlock;
  auxBlock.nzmax = sparseView.nzmax;
  auxBlock.m = sparseView.m;
  auxBlock.n = sparseView.n;
  auxBlock.p = sparseView.p;
  auxBlock.i = sparseView.i;
  auxBlock.x = nullptr;  // no values
  auxBlock.nz = -1;      // CCS format

  // AMD ordering on the block structure
  int* permutation = cs_amd(1, &auxBlock);
  if (!permutation) return false;
  result.resize(auxBlock.m);
  VectorXI::ConstMapType permutation_map(permutation, result.size());
  result = permutation_map;
  cs_free(permutation);
  return true;
}

CSparse::SparseView CSparse::sparseView() {
  CSparseExt& sparse = pImpl_->ccsA;
  return CSparse::SparseView(sparse.m, sparse.n, sparse.nzmax, sparse.p,
                             sparse.i, sparse.x, sparse.columnsAllocated);
}

CSparse::FactorView CSparse::factor() {
  csn* factor = pImpl_->numericCholesky;
  return CSparse::FactorView(factor->L->n, factor->L->p, factor->L->i,
                             factor->L->x, pImpl_->symbolicDecomposition->pinv);
}

bool CSparse::solve(double* x, double* b) const {
  pImpl_->prepareWorkspace();

  if (x != b) memcpy(x, b, pImpl_->ccsA.n * sizeof(double));
  const int ok = csparse_extension::cs_cholsolsymb(
      &pImpl_->ccsA, x, pImpl_->symbolicDecomposition, pImpl_->csWorkspace,
      pImpl_->csIntWorkspace);
  return static_cast<bool>(ok);
}

bool CSparse::analyze() {
  freeSymbolic();
  pImpl_->symbolicDecomposition = cs_schol(1, &pImpl_->ccsA);
  return pImpl_->symbolicDecomposition != nullptr;
}

bool CSparse::analyze_p(int* permutation) {
  freeSymbolic();
  pImpl_->symbolicDecomposition = static_cast<css*>(cs_calloc(1, sizeof(css)));
  const int n = pImpl_->ccsA.n;
  pImpl_->symbolicDecomposition->pinv = cs_pinv(permutation, n);
  cs* C = cs_symperm(&pImpl_->ccsA, pImpl_->symbolicDecomposition->pinv, 0);
  pImpl_->symbolicDecomposition->parent = cs_etree(C, 0);
  int* post = cs_post(pImpl_->symbolicDecomposition->parent, n);
  int* c = cs_counts(C, pImpl_->symbolicDecomposition->parent, post, 0);
  cs_free(post);
  cs_spfree(C);
  pImpl_->symbolicDecomposition->cp =
      static_cast<int*>(cs_malloc(n + 1, sizeof(int)));
  pImpl_->symbolicDecomposition->unz = pImpl_->symbolicDecomposition->lnz =
      cs_cumsum(pImpl_->symbolicDecomposition->cp, c, n);
  cs_free(c);
  if (pImpl_->symbolicDecomposition->lnz < 0) {
    cs_sfree(pImpl_->symbolicDecomposition);
    pImpl_->symbolicDecomposition = nullptr;
  }
  return pImpl_->symbolicDecomposition != nullptr;
}

int CSparse::choleskyNz() const {
  if (pImpl_->symbolicDecomposition) return pImpl_->symbolicDecomposition->lnz;
  return -1;
}

bool CSparse::factorize() {
  pImpl_->prepareWorkspace();
  freeFactor();
  pImpl_->numericCholesky = csparse_extension::cs_chol_workspace(
      &pImpl_->ccsA, pImpl_->symbolicDecomposition, pImpl_->csIntWorkspace,
      pImpl_->csWorkspace);

  return pImpl_->numericCholesky != nullptr;
}

bool CSparse::hasSymbolic() const {
  return pImpl_->symbolicDecomposition != nullptr;
}

void CSparse::freeSymbolic() {
  if (pImpl_->symbolicDecomposition != nullptr) {
    cs_sfree(pImpl_->symbolicDecomposition);
    pImpl_->symbolicDecomposition = nullptr;
  }
}

bool CSparse::writeSparse(const std::string& filename) const {
  return csparse_extension::writeCs2Octave(filename.c_str(), &pImpl_->ccsA,
                                           true);
}

}  // namespace csparse
}  // namespace g2o
