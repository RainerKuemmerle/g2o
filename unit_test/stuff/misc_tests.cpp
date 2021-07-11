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

#include "g2o/stuff/misc.h"
#include "gtest/gtest.h"

namespace {
number_t simple_normalize_theta(number_t th) {
  while (th >= g2o::const_pi()) th -= 2. * g2o::const_pi();
  while (th < -g2o::const_pi()) th += 2. * g2o::const_pi();
  return th;
}
}  // namespace

TEST(Stuff, NormalizeTheta) {
  constexpr double epsilon = 1e-9;
  EXPECT_NEAR(0, g2o::normalize_theta(0), epsilon);
  EXPECT_NEAR(M_PI, g2o::normalize_theta(M_PI), epsilon);
  EXPECT_NEAR(0, g2o::normalize_theta(2 * M_PI), epsilon);
  EXPECT_NEAR(M_PI, g2o::normalize_theta(3 * M_PI), epsilon);
  EXPECT_NEAR(0, g2o::normalize_theta(4 * M_PI), epsilon);

  EXPECT_NEAR(0, g2o::normalize_theta(-0), epsilon);
  EXPECT_NEAR(M_PI, g2o::normalize_theta(-M_PI), epsilon);
  EXPECT_NEAR(0, g2o::normalize_theta(-2 * M_PI), epsilon);
  EXPECT_NEAR(M_PI, g2o::normalize_theta(-3 * M_PI), epsilon);
  EXPECT_NEAR(0, g2o::normalize_theta(-4 * M_PI), epsilon);

  EXPECT_NEAR(0, g2o::normalize_theta(-0), epsilon);
  EXPECT_NEAR(-M_PI / 2, g2o::normalize_theta(-M_PI / 2), epsilon);
  EXPECT_NEAR(M_PI, g2o::normalize_theta(-M_PI), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::normalize_theta(-3 * M_PI / 2), epsilon);
  EXPECT_NEAR(0, g2o::normalize_theta(-4 * M_PI / 2), epsilon);

  EXPECT_NEAR(0, g2o::normalize_theta(0), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::normalize_theta(M_PI / 2), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::normalize_theta(5 * M_PI / 2), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::normalize_theta(9 * M_PI / 2), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::normalize_theta(-3 * M_PI / 2), epsilon);
}

TEST(Stuff, NormalizeThetaCmpBruteForce) {
  constexpr double epsilon = 1e-9;
  for (double d = -10.; d <= 10.; d += 0.1) {
    EXPECT_NEAR(simple_normalize_theta(d), g2o::normalize_theta(d), epsilon);
  }
}

TEST(Stuff, Deg2Rad) {
  constexpr double epsilon = 1e-9;
  EXPECT_NEAR(0, g2o::deg2rad(0), epsilon);
  EXPECT_NEAR(M_PI / 2, g2o::deg2rad(90), epsilon);
  EXPECT_NEAR(M_PI, g2o::deg2rad(180), epsilon);
  EXPECT_NEAR(M_PI * 3 / 2, g2o::deg2rad(270), epsilon);
  EXPECT_NEAR(2 * M_PI, g2o::deg2rad(360), epsilon);
  EXPECT_NEAR(M_PI / 3, g2o::deg2rad(60), epsilon);
  EXPECT_NEAR(M_PI * 2 / 3, g2o::deg2rad(120), epsilon);
  EXPECT_NEAR(M_PI / 4, g2o::deg2rad(45), epsilon);
  EXPECT_NEAR(M_PI * 3 / 4, g2o::deg2rad(135), epsilon);
  EXPECT_NEAR(M_PI / 6, g2o::deg2rad(30), epsilon);
}

TEST(Stuff, Rad2Deg) {
  constexpr double epsilon = 1e-9;
  EXPECT_NEAR(g2o::rad2deg(0), 0, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI / 2), 90, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI), 180, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI * 3 / 2), 270, epsilon);
  EXPECT_NEAR(g2o::rad2deg(2 * M_PI), 360, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI / 3), 60, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI * 2 / 3), 120, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI / 4), 45, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI * 3 / 4), 135, epsilon);
  EXPECT_NEAR(g2o::rad2deg(M_PI / 6), 30, epsilon);
}

TEST(Stuff, ArrayHasNaN) {
  constexpr int size = 10;
  number_t data[size];
  std::fill_n(data, size, 0);

  auto aux = [](const number_t* data, int size) {
    int nanIndex = -1;
    bool hasNan = g2o::arrayHasNaN(data, size, &nanIndex);
    return std::make_pair(hasNan, nanIndex);
  };

  EXPECT_EQ(std::make_pair(false, -1), aux(data, size));

  for (int i = 0; i < size; ++i) {
    std::fill_n(data, size, 0);
    data[i] = std::numeric_limits<number_t>::quiet_NaN();
    EXPECT_EQ(std::make_pair(true, i), aux(data, size));
  }
}

TEST(Stuff, Square) {
  EXPECT_DOUBLE_EQ(4, g2o::square(2));
  EXPECT_DOUBLE_EQ(4, g2o::square(-2));
}

TEST(Stuff, Hypot) {
  EXPECT_DOUBLE_EQ(hypot(2, 0), 2);
  EXPECT_DOUBLE_EQ(hypot(-2, 0), 2);
  EXPECT_DOUBLE_EQ(hypot(0, 2), 2);
  EXPECT_DOUBLE_EQ(hypot(0, -2), 2);
  EXPECT_DOUBLE_EQ(hypot(3, 3), sqrt(18));
  EXPECT_DOUBLE_EQ(hypot(-3, 3), sqrt(18));
  EXPECT_DOUBLE_EQ(hypot(-3, 3), sqrt(18));
  EXPECT_DOUBLE_EQ(hypot(3, -3), sqrt(18));
}
