#include "data_queue.h"

#include "g2o/types/data/robot_data.h"

namespace g2o {

  DataQueue::DataQueue()
  {
  }

  DataQueue::~DataQueue()
  {
  }

  RobotData* DataQueue::findClosestData(double timestamp) const
  {
    if (_buffer.rbegin()->first < timestamp)
      return _buffer.rbegin()->second;
    if (_buffer.begin()->first > timestamp)
      return _buffer.begin()->second;

    Buffer::const_iterator ub = _buffer.upper_bound(timestamp);
    Buffer::const_iterator lb = ub;
    --lb;
    if (fabs(lb->first - timestamp) < fabs(ub->first - timestamp))
      return lb->second;
    else
      return ub->second;
  }

  RobotData* DataQueue::before(double timestamp) const
  {
    if (_buffer.size() == 0 || _buffer.begin()->first > timestamp)
      return 0;
    Buffer::const_iterator lb = _buffer.upper_bound(timestamp);
    --lb; // now it's the lower bound
    return lb->second;
  }

  RobotData* DataQueue::after(double timestamp) const
  {
    if (_buffer.size() == 0 || _buffer.rbegin()->first < timestamp)
      return 0;
    Buffer::const_iterator ub = _buffer.upper_bound(timestamp);
    if (ub == _buffer.end())
      return 0;
    return ub->second;
  }

  void DataQueue::add(RobotData* rd)
  {
    _buffer[rd->timestamp()] = rd;
  }

} // end namespace
