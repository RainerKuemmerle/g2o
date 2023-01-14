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

#include "matrix_structure.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

namespace g2o {

struct ColSort {
  bool operator()(const std::pair<int, int>& e1,
                  const std::pair<int, int>& e2) const {
    return e1.second < e2.second ||
           (e1.second == e2.second && e1.first < e2.first);
  }
};

MatrixStructure::~MatrixStructure() { free(); }

void MatrixStructure::alloc(int n_, int nz) {
  if (n == 0) {
    maxN_ = n = n_;
    maxNz_ = nz;
    Ap = new int[maxN_ + 1];
    Aii = new int[maxNz_];
  } else {
    n = n_;
    if (maxNz_ < nz) {
      maxNz_ = 2 * nz;
      delete[] Aii;
      Aii = new int[maxNz_];
    }
    if (maxN_ < n) {
      maxN_ = 2 * n;
      delete[] Ap;
      Ap = new int[maxN_ + 1];
    }
  }
}

void MatrixStructure::free() {
  n = 0;
  m = 0;
  maxN_ = 0;
  maxNz_ = 0;
  delete[] Aii;
  Aii = nullptr;
  delete[] Ap;
  Ap = nullptr;
}

bool MatrixStructure::write(const char* filename) const {
  const int& cols = n;
  const int& rows = m;

  std::string name = filename;
  const std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos) name = name.substr(0, lastDot);

  std::vector<std::pair<int, int> > entries;
  for (int i = 0; i < cols; ++i) {
    const int& rbeg = Ap[i];
    const int& rend = Ap[i + 1];
    for (int j = rbeg; j < rend; ++j) {
      entries.emplace_back(Aii[j], i);
      if (Aii[j] != i) entries.emplace_back(i, Aii[j]);
    }
  }

  sort(entries.begin(), entries.end(), ColSort());

  std::ofstream fout(filename);
  fout << "# name: " << name << std::endl;
  fout << "# type: sparse matrix" << std::endl;
  fout << "# nnz: " << entries.size() << std::endl;
  fout << "# rows: " << rows << std::endl;
  fout << "# columns: " << cols << std::endl;
  for (const auto& entry : entries) {
    fout << entry.first << " " << entry.second << " 0"
         << std::endl;  // write a constant value of 0
  }

  return fout.good();
}

}  // namespace g2o
