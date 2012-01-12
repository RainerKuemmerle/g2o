// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "robot_data.h"

namespace g2o {

  RobotData::RobotData() :
    OptimizableGraph::Data(),
    _timestamp(-1.), _loggerTimestamp(-1.)
  {
  }

  RobotData::~RobotData()
  {
  }

  void RobotData::setTimestamp(double ts)
  {
    _timestamp = ts;
  }

  void RobotData::setLoggerTimestamp(double ts)
  {
    _loggerTimestamp = ts;
  }

  void RobotData::setTag(const std::string& tag)
  {
    _tag = tag;
  }

  void RobotData::setHostname(const std::string& hostname)
  {
    _hostname = hostname;
  }

} // end namespace
