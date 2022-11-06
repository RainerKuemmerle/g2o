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

#include <memory>
#include <vector>

#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_data.h"

class MyTrivialRobotData : public g2o::RobotData {
   bool write(std::ostream&) const override { return false; }
   bool read(std::istream&) override { return false; }
};

TEST(Data, DataQueue) {
  std::vector<std::shared_ptr<MyTrivialRobotData>> myRobotData;
  constexpr int kNumData = 10;
  constexpr double kTimeOffset = 1234567890.;
  for (int i = 0; i < kNumData; ++i) {
    auto data = std::make_shared<MyTrivialRobotData>();
    data->setTimestamp(kTimeOffset + i);
    myRobotData.emplace_back(data);
  }

  // create the robot queue
  g2o::DataQueue dataQueue;
  for (const auto& d : myRobotData) dataQueue.add(d);

  // TESTS
  ASSERT_EQ(nullptr, dataQueue.before(kTimeOffset));
  ASSERT_EQ(nullptr, dataQueue.after(kTimeOffset + kNumData - 1.));

  ASSERT_DOUBLE_EQ(kTimeOffset, dataQueue.findClosestData(0.)->timestamp());
  ASSERT_DOUBLE_EQ(
      kTimeOffset + kNumData - 1.,
      dataQueue.findClosestData(kTimeOffset + kNumData + 1)->timestamp());

  ASSERT_DOUBLE_EQ(kTimeOffset,
                   dataQueue.findClosestData(kTimeOffset)->timestamp());
  ASSERT_DOUBLE_EQ(kTimeOffset,
                   dataQueue.findClosestData(kTimeOffset + 0.45)->timestamp());
  ASSERT_DOUBLE_EQ(kTimeOffset + 1.,
                   dataQueue.findClosestData(kTimeOffset + 0.55)->timestamp());
  ASSERT_DOUBLE_EQ(kTimeOffset + 1.,
                   dataQueue.findClosestData(kTimeOffset + 1.)->timestamp());
}
