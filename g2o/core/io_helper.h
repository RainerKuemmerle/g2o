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

#include <Eigen/Core>
#include <iosfwd>
#include <istream>
#include <string>

namespace g2o {
namespace internal {
template <typename Derived>
bool writeVector(std::ostream& os, const Eigen::DenseBase<Derived>& b) {
  for (int i = 0; i < b.size(); i++) os << b(i) << " ";
  return os.good();
}

template <typename Derived>
bool readVector(std::istream& is, Eigen::DenseBase<Derived>& b) {
  for (int i = 0; i < b.size() && !is.fail(); i++) is >> b(i);
  return !is.fail();
}

/**
 * @brief Whether the current line holds another token.
 *
 * Consumes blanks up to the first non-blank character. Deliberately stops at
 * the end of the line: std::ws would consume the newline as well and let the
 * optional trailing fields of one record steal the tokens of the next one. A
 * trailing '\r' (CRLF input) counts as end of line.
 */
inline bool hasDataOnLine(std::istream& is) {
  using Traits = std::char_traits<char>;
  Traits::int_type c = is.peek();
  while (c == ' ' || c == '\t') {
    is.get();
    c = is.peek();
  }
  return c != '\n' && c != '\r' && !Traits::eq_int_type(c, Traits::eof());
}

/**
 * @brief Read optional trailing fields of a record.
 *
 * If the line ends here the fields are left untouched, so records written by
 * an older version of g2o keep whatever the constructor (or the caller) set.
 * Otherwise the fields are all-or-nothing: a short or garbled tail is an
 * error.
 */
template <typename... Ts>
bool readOptional(std::istream& is, Ts&... fields) {
  if (!hasDataOnLine(is)) return true;
  (is >> ... >> fields);
  return !is.fail();
}
}  // namespace internal
}  // namespace g2o

#endif
