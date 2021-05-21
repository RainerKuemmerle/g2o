// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_CORE_IO_HELPER_H
#define G2O_CORE_IO_HELPER_H

#include <iosfwd>
#include "graph.pb.h"

namespace g2o {
namespace internal {
template <typename Derived>
bool writeVector(std::ostream& os, const Eigen::DenseBase<Derived>& b) {
  for (int i = 0; i < b.size(); i++) os << b(i) << " ";
  return os.good();
}

template <typename Derived>
bool readVector(std::istream& is, Eigen::DenseBase<Derived>& b) {
  for (int i = 0; i < b.size() && is.good(); i++) is >> b(i);
  return is.good() || is.eof();
}
template <typename Derived>
bool writeVectorProto(g2o::proto::Row* row, const Eigen::DenseBase<Derived>& b) {
  for (int i = 0; i < b.size(); i++) {
    row->add_value(b(i));
  }
  return true;
}
template <typename Derived>
int readVectorProto(int idx, const g2o::proto::Row& row, Eigen::DenseBase<Derived>& b) {
  int advance = 0;
  for (int i = 0; i < b.size() && (i + idx) < row.value_size(); i++) {
    b(i) = row.value(i + idx);
    advance = advance + 1;
  }
  return advance;
}
}  // namespace internal
}  // namespace g2o

#endif
