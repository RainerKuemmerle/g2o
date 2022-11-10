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

#include "solver.h"

#include <algorithm>
#include <cstring>

#include "dynamic_aligned_buffer.hpp"

namespace g2o {

Solver::Solver() = default;

Solver::~Solver() {
  free_aligned(x_);
  free_aligned(b_);
}

void Solver::resizeVector(size_t sx) {
  const size_t oldSize = xSize_;
  xSize_ = sx;
  sx += additionalVectorSpace_;  // allocate some additional space if requested
  if (maxXSize_ < sx) {
    maxXSize_ = 2 * sx;
    free_aligned(x_);
    x_ = allocate_aligned<number_t>(maxXSize_);
#ifndef NDEBUG
    memset(x_, 0, maxXSize_ * sizeof(number_t));
#endif
    if (b_) {  // backup the former b, might still be needed for online
               // processing
      memcpy(x_, b_, oldSize * sizeof(number_t));
      free_aligned(b_);
      b_ = allocate_aligned<number_t>(maxXSize_);
      std::swap(b_, x_);
    } else {
      b_ = allocate_aligned<number_t>(maxXSize_);
#ifndef NDEBUG
      memset(b_, 0, maxXSize_ * sizeof(number_t));
#endif
    }
  }
}

void Solver::setOptimizer(SparseOptimizer* optimizer) {
  optimizer_ = optimizer;
}

void Solver::setLevenberg(bool levenberg) { isLevenberg_ = levenberg; }

void Solver::setAdditionalVectorSpace(size_t s) { additionalVectorSpace_ = s; }

}  // namespace g2o
