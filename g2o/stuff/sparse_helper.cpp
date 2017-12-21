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

#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace std;

namespace g2o {

  namespace {
    struct TripletEntry
    {
      int r, c;
      number_t x;
      TripletEntry(int r_, int c_, number_t x_) : r(r_), c(c_), x(x_) {}
    };
    struct TripletColSort
    {
      bool operator()(const TripletEntry& e1, const TripletEntry& e2) const
      {
        return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
      }
    };
  }

  bool writeVector(const string& filename, const number_t*v, int n)
  {
    ofstream os(filename.c_str());
    os << fixed;
    for (int i=0; i<n; i++)
      os << *v++ << endl;
    return os.good();
  }

  bool writeCCSMatrix(const string& filename, int rows, int cols, const int* Ap, const int* Ai, const number_t* Ax, bool upperTriangleSymmetric)
  {
    vector<TripletEntry> entries;
    entries.reserve((size_t)Ap[cols]);
    for (int i=0; i < cols; i++) {
      const int& rbeg = Ap[i];
      const int& rend = Ap[i+1];
      for (int j = rbeg; j < rend; j++) {
        entries.push_back(TripletEntry(Ai[j], i, Ax[j]));
        if (upperTriangleSymmetric && Ai[j] != i)
          entries.push_back(TripletEntry(i, Ai[j], Ax[j]));
      }
    }
    sort(entries.begin(), entries.end(), TripletColSort());

    string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) 
      name = name.substr(0, lastDot);

    std::ofstream fout(filename.c_str());
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << entries.size() << std::endl;
    fout << "# rows: " << rows << std::endl;
    fout << "# columns: " << cols << std::endl;
    //fout << fixed;
    fout << setprecision(9) << endl;
    for (vector<TripletEntry>::const_iterator it = entries.begin(); it != entries.end(); ++it) {
      const TripletEntry& entry = *it;
      fout << entry.r+1 << " " << entry.c+1 << " " << entry.x << std::endl;
    }
    return fout.good();
  }

} // end namespace
