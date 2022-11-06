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

#include "data_queue.h"

#include "g2o/types/data/robot_data.h"

namespace g2o {

DataQueue::RobotDataPtr DataQueue::findClosestData(number_t timestamp) const {
  if (buffer_.rbegin()->first < timestamp) return buffer_.rbegin()->second;
  if (buffer_.begin()->first > timestamp) return buffer_.begin()->second;

  auto ub = buffer_.upper_bound(timestamp);
  auto lb = ub;
  --lb;
  if (fabs(lb->first - timestamp) < fabs(ub->first - timestamp))
    return lb->second;
  return ub->second;
}

DataQueue::RobotDataPtr DataQueue::before(number_t timestamp) const {
  if (buffer_.empty() || buffer_.begin()->first >= timestamp) return nullptr;
  auto lb = buffer_.upper_bound(timestamp);
  --lb;  // now it's the lower bound
  return lb->second;
}

DataQueue::RobotDataPtr DataQueue::after(number_t timestamp) const {
  if (buffer_.empty() || buffer_.rbegin()->first < timestamp) return nullptr;
  auto ub = buffer_.upper_bound(timestamp);
  if (ub == buffer_.end()) return nullptr;
  return ub->second;
}

void DataQueue::add(DataQueue::RobotDataPtr rd) {
  buffer_.emplace(rd->timestamp(), rd);
}

}  // namespace g2o
