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

#ifndef G2O_CHOLMOD_WRAPPER_H
#define G2O_CHOLMOD_WRAPPER_H

#include <cstddef>
#include <memory>

namespace g2o {
namespace cholmod {

class Cholmod {
 public:
  Cholmod();
  ~Cholmod();

  //! View onto the sparse matrix structure of Cholmod using CCS storage
  struct SparseView {
    size_t& nrow;
    size_t& ncol;
    size_t& nzmax;
    int*& p;
    int*& i;
    double*& x;
    size_t& columnsAllocated;
    SparseView(size_t& nrow, size_t& ncol, size_t& nzmax, int*& p, int*& i,
               double*& x, size_t& columnsAllocated)
        : nrow(nrow),
          ncol(ncol),
          nzmax(nzmax),
          p(p),
          i(i),
          x(x),
          columnsAllocated(columnsAllocated) {}
  };

  //! View onto the cholesky factor
  struct FactorView {
    size_t& n;
    int*& p;
    int*& i;
    double*& x;
    int*& perm;
    FactorView(size_t& n, int*& p, int*& i, double*& x, int*& perm)
        : n(n), p(p), i(i), x(x), perm(perm) {}
  };

  bool factorize();
  bool hasFactor() const;
  void freeFactor();
  bool simplifyFactor();

  //! compute AMD ordering on the given SparseView, store into result
  bool amd(SparseView& sparseView, int* result);

  int choleskyNz() const;

  void solve(double* x, double* b) const;

  bool analyze();
  bool analyze_p(int* permutation);

  SparseView sparseView();
  FactorView factor();

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace cholmod
}  // namespace g2o
#endif
