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

#ifndef G2O_SPARSE_HELPER_H
#define G2O_SPARSE_HELPER_H

#include <string>

#include "g2o/config.h"
#include "g2o_stuff_api.h"

namespace g2o {

struct TripletEntry {
  int r, c;
  number_t x;
  TripletEntry(int r_, int c_, number_t x_) : r(r_), c(c_), x(x_) {}
};
struct TripletColSort {
  bool operator()(const TripletEntry& e1, const TripletEntry& e2) const {
    return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
  }
};

/**
 * write an array to a file, debugging
 */
G2O_STUFF_API bool writeVector(const std::string& filename, const number_t* v, int n);

/**
 * write a CCS matrix given by pointer to column, row, and values
 */
G2O_STUFF_API bool writeCCSMatrix(const std::string& filename, int rows, int cols, const int* p, const int* i,
                                  const double* v, bool upperTriangleSymmetric = true);

/**
 * write a triplet matrix given by pointers
 * @param filename filename to write to
 * @param nz number of elements
 * @param rows number of rows of the matrix
 * @param cols number of colmuns of the matrix
 * @param Ai pointer to the row index (nz elements)
 * @param Aj pointer to the column index (nz elements)
 * @param Ax pointer to the vlaues index (nz elements)
 */
G2O_STUFF_API bool writeTripletMatrix(const std::string& filename, int nz, int rows, int cols,
                                      const int* Ai, const int* Aj, const double* Ax,
                                      bool upperTriangleSymmetric = true);

}  // namespace g2o

#endif
