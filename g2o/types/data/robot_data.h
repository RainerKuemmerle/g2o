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

#ifndef G2O_ROBOT_DATA_H
#define G2O_ROBOT_DATA_H

#include <iosfwd>
#include <string>

#include "g2o/core/optimizable_graph.h"

namespace g2o {

  /**
   * \brief data recorded by the robot
   */
  class RobotData : public OptimizableGraph::Data
  {
    public:
      RobotData();
      virtual ~RobotData();

      double timestamp() const { return _timestamp;}
      void setTimestamp(double ts);

      double loggerTimestamp() const { return _loggerTimestamp;}
      void setLoggerTimestamp(double ts);

      const std::string& tag() const { return _tag;}
      void setTag(const std::string& tag);

      const std::string& hostname() const { return _hostname;}
      void setHostname(const std::string& hostname);

    protected:
      double _timestamp; ///< timestamp when the measurement was generated
      double _loggerTimestamp; ///< timestamp when the measurement was recorded
      std::string _tag; ///< string tag (FLASER, ROBOTLASER, ODOM..) of the line in the log
      std::string _hostname; ///< name of the computer/robot generating the data
  };

} // end namespace

#endif
