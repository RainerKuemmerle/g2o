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

#include <Eigen/Core>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/eigen_matcher.h"

using namespace testing;  // NOLINT

TEST(TypeTraits, VectorFixed) {
  using Traits = g2o::TypeTraits<g2o::Vector3>;
  EXPECT_THAT(Traits::kVectorDimension, Eq(3));
  EXPECT_THAT(Traits::kMinimalVectorDimension, Eq(3));
  EXPECT_THAT(Traits::kIsVector, Eq(1));
  EXPECT_THAT(Traits::kIsScalar, Eq(0));
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toVector(g2o::Vector3::Ones())),
              EigenEqual(g2o::internal::print_wrap(g2o::Vector3::Ones())));
  g2o::Vector3 data;
  Traits::toData(g2o::Vector3(1, 2, 3), data.data());
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::toVector(g2o::Vector3(1, 2, 3))),
      EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::toMinimalVector(g2o::Vector3::Ones())),
      EigenEqual(g2o::internal::print_wrap(g2o::Vector3::Ones())));
  Traits::toMinimalData(g2o::Vector3(0, 1, 2), data.data());
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::toVector(g2o::Vector3(0, 1, 2))),
      EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::fromVector(g2o::Vector3(0, 1, 2))),
      EigenEqual(g2o::internal::print_wrap(g2o::Vector3(0, 1, 2))));
  EXPECT_THAT(g2o::internal::print_wrap(
                  Traits::fromMinimalVector(g2o::Vector3(0, 1, 2))),
              EigenEqual(g2o::internal::print_wrap(g2o::Vector3(0, 1, 2))));
}

TEST(TypeTraits, VectorDynamic) {
  using Traits = g2o::TypeTraits<g2o::VectorX>;
  EXPECT_THAT(Traits::kVectorDimension, Eq(Eigen::Dynamic));
  EXPECT_THAT(Traits::kMinimalVectorDimension, Eq(Eigen::Dynamic));
  EXPECT_THAT(Traits::kIsVector, Eq(1));
  EXPECT_THAT(Traits::kIsScalar, Eq(0));
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::toVector(g2o::VectorX::Ones(4))),
      EigenEqual(g2o::internal::print_wrap(g2o::VectorX::Ones(4))));
  g2o::VectorX data(3);
  g2o::VectorX input(3);
  input << 1, 2, 3;
  Traits::toData(input, data.data());
  EXPECT_THAT(g2o::internal::print_wrap(input),
              EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::toMinimalVector(g2o::VectorX::Ones(4))),
      EigenEqual(g2o::internal::print_wrap(g2o::VectorX::Ones(4))));
  input << 0, 1, 2;
  Traits::toMinimalData(input, data.data());
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toVector(input)),
              EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(
      g2o::internal::print_wrap(Traits::fromVector(g2o::Vector3(0, 1, 2))),
      EigenEqual(g2o::internal::print_wrap(input)));
  EXPECT_THAT(g2o::internal::print_wrap(
                  Traits::fromMinimalVector(g2o::Vector3(0, 1, 2))),
              EigenEqual(g2o::internal::print_wrap(input)));
}

TEST(TypeTraits, Double) {
  using Vec1 = g2o::VectorN<1>;
  using Traits = g2o::TypeTraits<double>;
  EXPECT_THAT(Traits::kVectorDimension, Eq(1));
  EXPECT_THAT(Traits::kMinimalVectorDimension, Eq(1));
  EXPECT_THAT(Traits::kIsVector, Eq(0));
  EXPECT_THAT(Traits::kIsScalar, Eq(1));
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toVector(42.)),
              EigenEqual(g2o::internal::print_wrap(Vec1(42.))));
  Vec1 data;
  Traits::toData(3., data.data());
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toVector(3.)),
              EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toMinimalVector(5.)),
              EigenEqual(g2o::internal::print_wrap(Vec1(5.))));
  Traits::toMinimalData(1., data.data());
  EXPECT_THAT(g2o::internal::print_wrap(Traits::toVector(1.)),
              EigenEqual(g2o::internal::print_wrap(data)));
  EXPECT_THAT(Traits::fromVector(Vec1(2.)), Eq(2.));
  EXPECT_THAT(Traits::fromMinimalVector(Vec1(3.)), Eq(3.));
}

TEST(TypeTraits, Dimension) {
  EXPECT_THAT(g2o::DimensionTraits<g2o::Vector4>::dimension(g2o::Vector4()),
              Eq(4));
  EXPECT_THAT(g2o::DimensionTraits<g2o::VectorX>::dimension(g2o::VectorX(17)),
              Eq(17));
  EXPECT_THAT(g2o::DimensionTraits<double>::dimension(42.), Eq(1));
  EXPECT_THAT(
      g2o::DimensionTraits<g2o::Vector4>::minimalDimension(g2o::Vector4()),
      Eq(4));
  EXPECT_THAT(
      g2o::DimensionTraits<g2o::VectorX>::minimalDimension(g2o::VectorX(17)),
      Eq(17));
  EXPECT_THAT(g2o::DimensionTraits<double>::minimalDimension(42.), Eq(1));
}
