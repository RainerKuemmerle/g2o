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

#ifndef G2O_ROBOT_LASER_H
#define G2O_ROBOT_LASER_H

#include "raw_laser.h"
#include "g2o_types_data_api.h"

namespace g2o {

  /**
   * \brief laser measurement obtained by a robot
   *
   * A laser measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
   * the measurement was taken. The read/write function correspond to the CARMEN logfile format.
   */
  class G2O_TYPES_DATA_API RobotLaser : public RawLaser
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      RobotLaser();
      ~RobotLaser();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);

      SE2 laserPose() const { return _odomPose * _laserParams.laserPose;} 
      const SE2& odomPose() const { return _odomPose;}
      void setOdomPose(const SE2& odomPose);

    protected:
      SE2 _odomPose;
      //! velocities and safety distances of the robot.
      double _laserTv, _laserRv, _forwardSafetyDist, _sideSaftyDist, _turnAxis;
  };

}

#endif
