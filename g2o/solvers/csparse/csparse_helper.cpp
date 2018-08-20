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

#include "csparse_helper.h"

#include "g2o/stuff/macros.h"

#include <string>
#include <cassert>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

namespace g2o {
namespace csparse_extension {

  struct SparseMatrixEntry{
    SparseMatrixEntry(int r=-1, int c=-1, number_t x=0) :
      _r(r), _c(c), _x(x)
    {
    }
    int _r,_c;
    number_t _x;
  };

  struct SparseMatrixEntryColSort
  {
    bool operator()(const SparseMatrixEntry& e1, const SparseMatrixEntry& e2) const
    {
      return e1._c < e2._c || (e1._c == e2._c && e1._r < e2._r);
    }
  };

  bool writeCs2Octave(const char* filename, const cs* A, bool upperTriangular)
  {
    int cols = A->n;
    int rows = A->m;

    string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) 
      name = name.substr(0, lastDot);

    vector<SparseMatrixEntry> entries;
    if (A->nz == -1) { // CCS matrix
      const int* Ap = A->p;
      const int* Ai = A->i;
      const number_t* Ax = A->x;
      for (int i=0; i < cols; i++) {
        const int& rbeg = Ap[i];
        const int& rend = Ap[i+1];
        for (int j = rbeg; j < rend; j++) {
          entries.push_back(SparseMatrixEntry(Ai[j], i, Ax[j]));
          if (upperTriangular && Ai[j] != i)
            entries.push_back(SparseMatrixEntry(i, Ai[j], Ax[j]));
        }
      }
    } else { // Triplet matrix
      entries.reserve(A->nz);
      int *Aj = A->p;             // column indeces
      int *Ai = A->i;             // row indices
      number_t *Ax = A->x;          // values;
      for (int i = 0; i < A->nz; ++i) {
        entries.push_back(SparseMatrixEntry(Ai[i], Aj[i], Ax[i]));
        if (upperTriangular && Ai[i] != Aj[i])
          entries.push_back(SparseMatrixEntry(Aj[i], Ai[i], Ax[i]));
      }
    }
    sort(entries.begin(), entries.end(), SparseMatrixEntryColSort());

    std::ofstream fout(filename);
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << entries.size() << std::endl;
    fout << "# rows: " << rows << std::endl;
    fout << "# columns: " << cols << std::endl;
    //fout << fixed;
    fout << setprecision(9) << endl;

    for (vector<SparseMatrixEntry>::const_iterator it = entries.begin(); it != entries.end(); ++it) {
      const SparseMatrixEntry& entry = *it;
      fout << entry._r+1 << " " << entry._c+1 << " " << entry._x << std::endl;
    }
    return fout.good();
  }

} // end namespace
} // end namespace
