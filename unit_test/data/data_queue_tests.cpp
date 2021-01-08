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

#include <vector>

#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_data.h"

using namespace std;
using namespace g2o;

class MyTrivialRobotData : public RobotData {
  virtual bool write(std::ostream&) const { return false; }
  virtual bool read(std::istream&) { return false; }
};

TEST(Data, DataQueue) {
  vector<RobotData*> myRobotData;
  constexpr int knumData = 10;
  constexpr double ktimeOffset = 1234567890.;
  for (int i = 0; i < knumData; ++i) {
    RobotData* data = new MyTrivialRobotData();
    data->setTimestamp(ktimeOffset + i);
    myRobotData.emplace_back(data);
  }

  // create the robot queue
  DataQueue dataQueue;
  for (auto d : myRobotData) dataQueue.add(d);

  // TESTS
  ASSERT_EQ(nullptr, dataQueue.before(ktimeOffset));
  ASSERT_EQ(nullptr, dataQueue.after(ktimeOffset + knumData - 1.));

  ASSERT_DOUBLE_EQ(ktimeOffset, dataQueue.findClosestData(0.)->timestamp());
  ASSERT_DOUBLE_EQ(ktimeOffset + knumData - 1.,
                   dataQueue.findClosestData(ktimeOffset + knumData + 1)->timestamp());

  ASSERT_DOUBLE_EQ(ktimeOffset, dataQueue.findClosestData(ktimeOffset)->timestamp());
  ASSERT_DOUBLE_EQ(ktimeOffset, dataQueue.findClosestData(ktimeOffset + 0.45)->timestamp());
  ASSERT_DOUBLE_EQ(ktimeOffset + 1., dataQueue.findClosestData(ktimeOffset + 0.55)->timestamp());
  ASSERT_DOUBLE_EQ(ktimeOffset + 1., dataQueue.findClosestData(ktimeOffset + 1.)->timestamp());

  // clean up
  for (auto d : myRobotData) delete d;
}
