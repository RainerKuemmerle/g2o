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

#include "gtest/gtest.h"

#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/slam3d/edge_se3.h"

using namespace std;

TEST(MappingsSlam3D, EulerConversion)
{
  g2o::Vector3 eulerAngles(.1,.2,.3);
  g2o::Matrix3 m1 = g2o::internal::fromEuler(eulerAngles);
  g2o::Vector3 eulerAnglesFromMatrix = g2o::internal::toEuler(m1);
  for (int i = 0; i < 3; ++i)
    EXPECT_DOUBLE_EQ(eulerAngles(i), eulerAnglesFromMatrix(i));
}

TEST(MappingsSlam3D, QuaternionConversion)
{
  g2o::Vector3 eulerAngles(.1,.2,.3);
  g2o::Matrix3 m1 = g2o::internal::fromEuler(eulerAngles);
  g2o::Vector3 q = g2o::internal::toCompactQuaternion(m1);
  g2o::Matrix3 m2 = g2o::internal::fromCompactQuaternion(q);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      EXPECT_DOUBLE_EQ(m1(r,c), m2(r,c));
}

TEST(MappingsSlam3D, ET)
{
  g2o::Vector3 eulerAngles(.1,.2,.3);
  g2o::Vector6 et;
  g2o::Vector3 t(1.,2.,3.);
  et.block<3,1>(0,0) = t;
  et.block<3,1>(3,0) = eulerAngles;
  g2o::Matrix3 m1 = g2o::internal::fromEuler(eulerAngles);

  g2o::Isometry3 i1 = g2o::internal::fromVectorET(et);
  for (int r = 0; r < 3; ++r)
    EXPECT_EQ(t(r), i1.translation()(r));

  EXPECT_NEAR(0., (i1.linear() - m1).array().abs().maxCoeff(), 1e-6);

  g2o::Vector6 et2 = g2o::internal::toVectorET(i1);
  EXPECT_NEAR(0., (et - et2).array().abs().maxCoeff(), 1e-6);
}

TEST(MappingsSlam3D, MQT)
{
  g2o::Vector3 eulerAngles(.1,.2,.3);
  g2o::Vector6 et;
  g2o::Vector3 t(1.,2.,3.);
  et.block<3,1>(0,0) = t;
  et.block<3,1>(3,0) = eulerAngles;
  g2o::Isometry3 i1 = g2o::internal::fromVectorET(et);

  g2o::Vector6 qt1 = g2o::internal::toVectorMQT(i1);

  g2o::Isometry3 i2 = g2o::internal::fromVectorMQT(qt1);
  EXPECT_NEAR(0., (i1.linear() - i2.linear()).array().abs().maxCoeff(), 1e-6);
  EXPECT_NEAR(0., (i1.translation() - i2.translation()).array().abs().maxCoeff(), 1e-6);
}

TEST(MappingsSlam3D, QT)
{
  g2o::Vector3 eulerAngles(.1,.2,.3);
  g2o::Vector6 et;
  g2o::Vector3 t(1.,2.,3.);
  et.block<3,1>(0,0) = t;
  et.block<3,1>(3,0) = eulerAngles;
  g2o::Isometry3 i1 = g2o::internal::fromVectorET(et);

  g2o::Vector7 qt2 = g2o::internal::toVectorQT(i1);

  g2o::Isometry3 i2 = g2o::internal::fromVectorQT(qt2);
  EXPECT_NEAR(0., (i1.linear() - i2.linear()).array().abs().maxCoeff(), 1e-6);
  EXPECT_NEAR(0., (i1.translation() - i2.translation()).array().abs().maxCoeff(), 1e-6);
}
