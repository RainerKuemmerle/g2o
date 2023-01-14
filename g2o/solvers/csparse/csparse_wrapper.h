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

#ifndef G2O_CSPARSE_WRAPPER_H
#define G2O_CSPARSE_WRAPPER_H

#include <cstddef>
#include <memory>

#include "g2o/core/eigen_types.h"

namespace g2o {
namespace csparse {

class CSparse {
 public:
  CSparse();
  ~CSparse();

  //! View onto the sparse matrix structure of CSparse using CCS storage
  struct SparseView {
    int& m;
    int& n;
    int& nzmax;
    int*& p;
    int*& i;
    double*& x;
    int& columnsAllocated;
    SparseView(int& m, int& n, int& nzmax, int*& p, int*& i, double*& x,
               int& columnsAllocated)
        : m(m),
          n(n),
          nzmax(nzmax),
          p(p),
          i(i),
          x(x),
          columnsAllocated(columnsAllocated) {}
  };

  //! View onto the cholesky factor
  struct FactorView {
    int& n;
    int*& p;
    int*& i;
    double*& x;
    int*& pinv;
    FactorView(int& n, int*& p, int*& i, double*& x, int*& pinv)
        : n(n), p(p), i(i), x(x), pinv(pinv) {}
  };

  bool factorize();
  bool hasFactor() const;
  void freeFactor();
  FactorView factor();

  //! compute AMD ordering on the given SparseView, store into result
  static bool amd(const SparseView& sparseView, VectorXI& result);

  int choleskyNz() const;

  bool solve(double* x, double* b) const;

  bool analyze();
  bool analyze_p(int* permutation);

  bool hasSymbolic() const;
  void freeSymbolic();

  SparseView sparseView();
  bool writeSparse(const std::string& filename) const;

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace csparse
}  // namespace g2o
#endif
