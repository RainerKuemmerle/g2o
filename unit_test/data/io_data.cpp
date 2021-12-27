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

#include <gmock/gmock.h>

#include <sstream>

#include "g2o/stuff/sampler.h"
#include "g2o/types/data/robot_laser.h"

using namespace std;
using namespace g2o;
using namespace testing;

MATCHER(NearEq, "") { return fabs(std::get<0>(arg) - std::get<1>(arg)) < 0.01; }

MATCHER(PointsNearEq, "") {
  return (std::get<0>(arg) - std::get<1>(arg)).norm() < 0.01;
}

TEST(Data, ReadWriteRobotLaser) {
  constexpr int kNumBeams = 180;

  vector<number_t> ranges;
  vector<number_t> remissions;
  for (int i = 0; i < kNumBeams; ++i) {
    ranges.push_back(i * 0.1);
    remissions.push_back(sampleUniform(0., 1.));
  }

  LaserParameters laserParams(kNumBeams, -M_PI_2, 1.0, 20.0);
  RobotLaser laser;
  laser.setLaserParams(laserParams);
  laser.setRanges(ranges);
  laser.setRemissions(remissions);
  laser.setOdomPose(SE2(1., 2., 0.34));

  // write the data to read again
  stringstream dataStream;
  laser.write(dataStream);

  RobotLaser recoveredLaser;
  recoveredLaser.read(dataStream);

  ASSERT_THAT(recoveredLaser.ranges(), SizeIs(laser.ranges().size()));
  ASSERT_THAT(recoveredLaser.remissions(), SizeIs(laser.remissions().size()));
  ASSERT_TRUE(recoveredLaser.odomPose().toVector().isApprox(
      laser.odomPose().toVector()));
  ASSERT_TRUE(recoveredLaser.laserPose().toVector().isApprox(
      laser.laserPose().toVector()));

  ASSERT_THAT(recoveredLaser.ranges(), Pointwise(NearEq(), laser.ranges()));
  ASSERT_THAT(recoveredLaser.remissions(),
              Pointwise(NearEq(), laser.remissions()));
  ASSERT_THAT(recoveredLaser.cartesian(),
              Pointwise(PointsNearEq(), laser.cartesian()));
}
