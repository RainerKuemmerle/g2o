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

#include "cholmod_wrapper.h"

#include <cholmod.h>

#include <cassert>
#include <cstring>
#include <memory>

#include "cholmod_ext.h"

namespace g2o {
namespace cholmod {

class Cholmod::Impl {
 public:
  Impl() {
    cholmod_start(&cholmodCommon);

    // setup ordering strategy
    cholmodCommon.nmethods = 1;
    cholmodCommon.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
    //_cholmodCommon.postorder = 0;

    cholmodCommon.supernodal =
        CHOLMOD_AUTO;  // CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
  }
  ~Impl() {
    freeFactor();
    cholmod_finish(&cholmodCommon);
  }

  void freeFactor() {
    if (cholmodFactor != nullptr) {
      cholmod_free_factor(&cholmodFactor, &cholmodCommon);
      cholmodFactor = nullptr;
    }
  }

  cholmod_common cholmodCommon;
  CholmodExt cholmodSparse;
  cholmod_factor* cholmodFactor = nullptr;
};

Cholmod::Cholmod() : pImpl(std::make_unique<Impl>()) {}

Cholmod::~Cholmod() = default;

void Cholmod::freeFactor() { pImpl->freeFactor(); }

bool Cholmod::hasFactor() const { return pImpl->cholmodFactor != nullptr; }

bool Cholmod::amd(SparseView& sparseView, int* result) {
  cholmod_sparse auxCholmodSparse;
  auxCholmodSparse.nzmax = sparseView.nzmax;
  auxCholmodSparse.nrow = sparseView.nrow;
  auxCholmodSparse.ncol = sparseView.ncol;
  auxCholmodSparse.p = sparseView.p;
  auxCholmodSparse.i = sparseView.i;
  auxCholmodSparse.nz = 0;
  auxCholmodSparse.x = 0;
  auxCholmodSparse.z = 0;
  auxCholmodSparse.stype = 1;
  auxCholmodSparse.xtype = CHOLMOD_PATTERN;
  auxCholmodSparse.itype = CHOLMOD_INT;
  auxCholmodSparse.dtype = CHOLMOD_DOUBLE;
  auxCholmodSparse.sorted = 1;
  auxCholmodSparse.packed = 1;

  int amdStatus =
      cholmod_amd(&auxCholmodSparse, NULL, 0, result, &pImpl->cholmodCommon);
  return amdStatus != 0;
}

Cholmod::SparseView Cholmod::sparseView() {
  CholmodExt& sparse = pImpl->cholmodSparse;
  return Cholmod::SparseView(
      sparse.nrow, sparse.ncol, sparse.nzmax,
      *reinterpret_cast<int**>(&sparse.p), *reinterpret_cast<int**>(&sparse.i),
      *reinterpret_cast<double**>(&sparse.x), sparse.columnsAllocated);
}

Cholmod::FactorView Cholmod::factor() {
  cholmod_factor& factor = *pImpl->cholmodFactor;
  return Cholmod::FactorView(factor.n, *reinterpret_cast<int**>(&factor.p),
                             *reinterpret_cast<int**>(&factor.i),
                             *reinterpret_cast<double**>(&factor.x),
                             *reinterpret_cast<int**>(&factor.Perm));
}

void Cholmod::solve(double* x, double* b) const {
  // setting up b for calling cholmod
  cholmod_dense bcholmod;
  bcholmod.nrow = bcholmod.d = pImpl->cholmodSparse.nrow;
  bcholmod.ncol = 1;
  bcholmod.x = b;
  bcholmod.xtype = CHOLMOD_REAL;
  bcholmod.dtype = CHOLMOD_DOUBLE;
  cholmod_dense* xcholmod = cholmod_solve(CHOLMOD_A, pImpl->cholmodFactor,
                                          &bcholmod, &pImpl->cholmodCommon);
  std::memcpy(x, xcholmod->x,
              sizeof(double) * bcholmod.nrow);  // copy back to our array
  cholmod_free_dense(&xcholmod, &pImpl->cholmodCommon);
}

bool Cholmod::analyze() {
  // setup ordering strategy
  pImpl->cholmodCommon.nmethods = 1;
  pImpl->cholmodCommon.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
  pImpl->cholmodFactor =
      cholmod_analyze(&pImpl->cholmodSparse,
                      &pImpl->cholmodCommon);  // symbolic factorization
  return true;
}

bool Cholmod::analyze_p(int* permutation) {
  pImpl->cholmodCommon.nmethods = 1;
  pImpl->cholmodCommon.method[0].ordering = CHOLMOD_GIVEN;
  pImpl->cholmodFactor = cholmod_analyze_p(&pImpl->cholmodSparse, permutation,
                                           NULL, 0, &pImpl->cholmodCommon);
  return true;
}

int Cholmod::choleskyNz() const {
  return static_cast<int>(pImpl->cholmodCommon.method[0].lnz);
}

bool Cholmod::factorize() {
  cholmod_factorize(&pImpl->cholmodSparse, pImpl->cholmodFactor,
                    &pImpl->cholmodCommon);
  return pImpl->cholmodCommon.status == CHOLMOD_OK;
}

bool Cholmod::simplifyFactor() {
  // convert the factorization to LL, simplical, packed, monotonic
  int change_status = cholmod_change_factor(
      CHOLMOD_REAL, 1, 0, 1, 1, pImpl->cholmodFactor, &pImpl->cholmodCommon);
  assert(pImpl->cholmodFactor->is_ll && !pImpl->cholmodFactor->is_super &&
         pImpl->cholmodFactor->is_monotonic &&
         "Cholesky factor has wrong format");
  return change_status != 0;
}

}  // namespace cholmod
}  // namespace g2o
