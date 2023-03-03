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

#include <regex>
#include <sstream>

#include "g2o/core/batch_stats.h"
#include "gtest/gtest.h"

namespace {
std::string extractValue(const std::string& input, const std::string& field) {
  const std::regex field_regex("(" + field + ")= ([^\t]*)");
  std::smatch match;
  if (!std::regex_search(input, match, field_regex)) {
    return "";
  }
  if (match.size() != 3) {
    return "";
  }
  return match[2].str();
}
}  // namespace

TEST(General, BatchStatisticsOutput) {
  g2o::G2OBatchStatistics stats;

  std::ostringstream output_stream;
  stats.iteration = 42;
  stats.choleskyNNZ = 1001;
  stats.timeMarginals = 12.1;
  stats.levenbergIterations = 4;
  output_stream << stats;

  EXPECT_EQ("42", extractValue(output_stream.str(), "iteration"));
  EXPECT_EQ("1001", extractValue(output_stream.str(), "choleskyNNZ"));
  EXPECT_EQ("12.1", extractValue(output_stream.str(), "timeMarginals"));
  EXPECT_EQ("4", extractValue(output_stream.str(), "levenbergIterations"));
}
