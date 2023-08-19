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

#include "g2o/types/slam2d/se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_matcher.h"

using namespace g2o;      // NOLINT
using namespace testing;  // NOLINT

TEST(MappingsSlam2D, SE2) {
  // constructor from 3 values
  const SE2 s1(cst(0.), cst(1.), cst(2.));
  EXPECT_DOUBLE_EQ(0., s1[0]);
  EXPECT_DOUBLE_EQ(1., s1[1]);
  EXPECT_DOUBLE_EQ(2., s1[2]);

  // constructor from vector
  const Vector3 vec2 = Vector3(cst(0.1), cst(0.2), cst(0.3));
  const SE2 s2(vec2);
  EXPECT_DOUBLE_EQ(0.1, s2[0]);
  EXPECT_DOUBLE_EQ(0.2, s2[1]);
  EXPECT_DOUBLE_EQ(0.3, s2[2]);
  EXPECT_DOUBLE_EQ(0., (vec2.head<2>() - s2.translation()).norm());

  // constructor from Isometry
  const Rotation2D rot3 = Rotation2D(cst(1.));
  const SE2 s3 = SE2(Isometry2(rot3));
  EXPECT_DOUBLE_EQ(0., s3.translation()(0));
  EXPECT_DOUBLE_EQ(0., s3.translation()(1));
  EXPECT_DOUBLE_EQ(1., s3.rotation().angle());
}

TEST(MappingsSlam2D, TraitsSE2) {
  using Traits = TypeTraits<SE2>;
  EXPECT_THAT(Traits::kVectorDimension, Eq(3));
  EXPECT_THAT(Traits::kMinimalVectorDimension, Eq(3));
  EXPECT_THAT(Traits::kIsVector, Eq(0));
  EXPECT_THAT(Traits::kIsScalar, Eq(0));
  EXPECT_THAT(print_wrap(Traits::toVector(SE2())),
              EigenEqual(print_wrap(Vector3::Zero())));
  Vector3 data;
  Traits::toData(SE2(1, 2, 3), data.data());
  EXPECT_THAT(print_wrap(Traits::toVector(SE2(1, 2, 3))),
              EigenEqual(print_wrap(data)));
  EXPECT_THAT(print_wrap(Traits::toMinimalVector(SE2(1, 1, 1))),
              EigenEqual(print_wrap(Vector3::Ones())));
  Traits::toMinimalData(SE2(0, 1, 2), data.data());
  EXPECT_THAT(print_wrap(Traits::toVector(SE2(0, 1, 2))),
              EigenEqual(print_wrap(data)));
  EXPECT_THAT(print_wrap(Traits::fromVector(Vector3(0, 1, 2)).translation()),
              EigenEqual(print_wrap(Vector2(0, 1))));
  EXPECT_THAT(Traits::fromVector(Vector3(0, 1, 2)).rotation().angle(), Eq(2.));
  EXPECT_THAT(
      print_wrap(Traits::fromMinimalVector(Vector3(0, 1, 2)).translation()),
      EigenEqual(print_wrap(Vector2(0, 1))));
  EXPECT_THAT(print_wrap(Traits::Identity().toVector()),
              EigenEqual(print_wrap(Vector3::Zero())));
}
