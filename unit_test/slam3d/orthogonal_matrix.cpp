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

using namespace std;

TEST(Slam3D, OrthogonalMatrix)
{
  g2o::Matrix3 shouldBeIdentity;
  g2o::Matrix3 R = g2o::Matrix3::Identity();
  g2o::Matrix3 rot = (g2o::Matrix3)g2o::AngleAxis(0.01, g2o::Vector3::UnitZ());
  rot = rot * (g2o::Matrix3)g2o::AngleAxis(0.01, g2o::Vector3::UnitX());
  
  shouldBeIdentity = R * R.transpose();
  number_t initialDifference = (shouldBeIdentity - g2o::Matrix3::Identity()).array().abs().maxCoeff();
  EXPECT_DOUBLE_EQ(0., initialDifference);

  //introduce numerical inaccuracies
  for (int i = 0; i < 10000; ++i)
    R = R * rot;
  shouldBeIdentity = R * R.transpose();
  number_t afterMultDifference = (shouldBeIdentity - g2o::Matrix3::Identity()).array().abs().maxCoeff();
  EXPECT_GE(afterMultDifference, initialDifference);

  number_t inaccurateDet = R.determinant();
  g2o::Matrix3 approxSolution = R;
  g2o::internal::approximateNearestOrthogonalMatrix(approxSolution);
  g2o::internal::nearestOrthogonalMatrix(R);

  EXPECT_LE(std::abs(R.determinant() - 1.), std::abs(inaccurateDet - 1.));
  EXPECT_NEAR(1.0, R.determinant(), 1e-7);
  shouldBeIdentity = R * R.transpose();
  number_t nearestDifference = (shouldBeIdentity - g2o::Matrix3::Identity()).array().abs().maxCoeff();
  EXPECT_NEAR(0., nearestDifference, 1e-7);

  // norm of the comluns
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(1.0, R.col(i).norm(), 1e-7);

  shouldBeIdentity = approxSolution * approxSolution.transpose();
  number_t approxNearestDifference = (shouldBeIdentity - Eigen::Matrix3d::Identity()).array().abs().maxCoeff();
  EXPECT_NEAR(0., approxNearestDifference, 1e-6);
  EXPECT_LE(std::abs(R.determinant() - 1.), std::abs(approxSolution.determinant() - 1.));
  EXPECT_NEAR(1.0, approxSolution.determinant(), 1e-6);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(1.0, approxSolution.col(i).norm(), 1e-6);
}
