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

#include "g2o/stuff/tuple_tools.h"
#include "gtest/gtest.h"

TEST(Stuff, TupleApply) {
  auto t = std::make_tuple(1, 2, 3);
  ASSERT_EQ(1, std::get<0>(t));
  ASSERT_EQ(2, std::get<1>(t));
  ASSERT_EQ(3, std::get<2>(t));
  auto plus_one = [](int& i) { ++i; };
  g2o::tuple_apply_i(plus_one, t, 1);
  EXPECT_EQ(1, std::get<0>(t));
  EXPECT_EQ(3, std::get<1>(t));
  EXPECT_EQ(3, std::get<2>(t));
}

TEST(Stuff, TupleSetI) {
  auto t = std::make_tuple(1, 2, 3);
  g2o::tuple_set_i(5, t, 0);
  EXPECT_EQ(5, std::get<0>(t));
  EXPECT_EQ(2, std::get<1>(t));
  EXPECT_EQ(3, std::get<2>(t));
}

TEST(Stuff, TupleGetI) {
  auto t = std::make_tuple(1, 2, 3);
  int i0, i1, i2;  // NOLINT
  g2o::tuple_get_i(i0, t, 0);
  g2o::tuple_get_i(i1, t, 1);
  g2o::tuple_get_i(i2, t, 2);
  EXPECT_EQ(i0, std::get<0>(t));
  EXPECT_EQ(i1, std::get<1>(t));
  EXPECT_EQ(i2, std::get<2>(t));
}
