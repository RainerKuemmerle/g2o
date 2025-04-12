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

#include "sparse_helper.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace std;

namespace g2o {

static bool writeTripletEntries(const std::string& filename, int rows, int cols,
                                const std::vector<TripletEntry>& triplets) {
  const string name = [&filename]() {
    const std::string::size_type lastDot = filename.find_last_of('.');
    if (lastDot != std::string::npos) return filename.substr(0, lastDot);
    return filename;
  }();

  std::ofstream fout(filename.c_str());
  fout << "# name: " << name << std::endl;
  fout << "# type: sparse matrix" << std::endl;
  fout << "# nnz: " << triplets.size() << std::endl;
  fout << "# rows: " << rows << std::endl;
  fout << "# columns: " << cols << std::endl;
  // fout << fixed;
  fout << setprecision(9) << endl;
  for (const TripletEntry& entry : triplets) {
    fout << entry.r + 1 << " " << entry.c + 1 << " " << entry.x << std::endl;
  }
  return fout.good();
}

bool writeVector(const string& filename, const double* v, int n) {
  ofstream os(filename.c_str());
  os << fixed;
  for (int i = 0; i < n; i++) os << *v++ << endl;
  return os.good();
}

bool writeCCSMatrix(const string& filename, int rows, int cols, const int* Ap,
                    const int* Ai, const double* Ax,
                    bool upperTriangleSymmetric) {
  vector<TripletEntry> entries;
  entries.reserve((size_t)Ap[cols]);
  for (int i = 0; i < cols; i++) {
    const int& rbeg = Ap[i];
    const int& rend = Ap[i + 1];
    for (int j = rbeg; j < rend; j++) {
      entries.emplace_back(TripletEntry(Ai[j], i, Ax[j]));
      if (upperTriangleSymmetric && Ai[j] != i)
        entries.emplace_back(TripletEntry(i, Ai[j], Ax[j]));
    }
  }
  sort(entries.begin(), entries.end(), TripletColSort());
  return writeTripletEntries(filename, rows, cols, entries);
}

bool writeTripletMatrix(const std::string& filename, int nz, int rows, int cols,
                        const int* Ai, const int* Aj, const double* Ax,
                        bool upperTriangleSymmetric) {
  vector<TripletEntry> entries;
  entries.reserve(nz);
  for (int i = 0; i < nz; ++i) {
    entries.emplace_back(TripletEntry(Ai[i], Aj[i], Ax[i]));
    if (upperTriangleSymmetric && Ai[i] != Aj[i])
      entries.emplace_back(TripletEntry(Aj[i], Ai[i], Ax[i]));
  }
  sort(entries.begin(), entries.end(), TripletColSort());
  return writeTripletEntries(filename, rows, cols, entries);
}

}  // namespace g2o
