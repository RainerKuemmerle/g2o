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

#include "g2o/types/icp/types_icp.h"
#include "gtest/gtest.h"

using namespace g2o;

void checkRotationMatrix(const Matrix3& R) {
  constexpr double tol = 1e-14;
  // check norms of basis vectors
  EXPECT_NEAR(R.row(0).norm(), 1, tol);
  EXPECT_NEAR(R.row(1).norm(), 1, tol);
  EXPECT_NEAR(R.row(2).norm(), 1, tol);
  // check orthogonality
  EXPECT_NEAR(R.row(0).dot(R.row(1)), 0, tol);
  EXPECT_NEAR(R.row(1).dot(R.row(2)), 0, tol);
  EXPECT_NEAR(R.row(2).dot(R.row(0)), 0, tol);
  // check that basis is left-handed
  EXPECT_NEAR(R.determinant(), -1.0, tol);
}

/*
 * ROTATION MATRIX Tests
 */
TEST(IcpRotation, RotationMatrix) {
  constexpr size_t thetaPoints = 100;
  constexpr size_t phiPoints = 100;
  EdgeGICP edge;
  for (size_t n = 0; n < thetaPoints; n++) {
    const double theta = M_PI * n / thetaPoints;
    for (size_t k = 0; k < phiPoints; k++) {
      const double phi = 2.0L * M_PI * k / phiPoints;
      const Vector3 normal(std::sin(theta) * std::cos(phi), std::cos(theta),
                           std::sin(theta) * std::sin(phi));
      edge.normal0 = normal;
      edge.normal1 = normal;

      edge.makeRot0();
      edge.makeRot1();

      checkRotationMatrix(edge.R0);
      checkRotationMatrix(edge.R1);
    }
  }
}
