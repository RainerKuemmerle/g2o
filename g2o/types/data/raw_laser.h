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

#ifndef G2O_RAW_LASER_H
#define G2O_RAW_LASER_H

#include "robot_data.h"
#include "laser_parameters.h"

#include <vector>

#include<Eigen/Core>
#include<Eigen/StdVector>

namespace g2o {

  /**
   * \brief Raw laser measuerement
   *
   * A raw laser measuerement. The read/write function correspond to the format of CARMEN.
   */
  class RawLaser : public RobotData {
    public:
      typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >      Point2DVector;

    public:
      RawLaser();
      ~RawLaser();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);

      /**
       * computes a cartesian view of the beams (x,y).
       * @return a vector with the points of the scan in cartesian coordinates.
       */
      Point2DVector cartesian() const;

      //! the range measurements by the laser
      const std::vector<double>& ranges() const { return _ranges;}
      void setRanges(const std::vector<double>& ranges);

      //! the remission measurements by the laser
      const std::vector<double>& remissions() const { return _remissions;}
      void setRemissions(const std::vector<double>& remissions);

      //! the parameters of the laser
      const LaserParameters& laserParams() const { return _laserParams;}
      void setLaserParams(const LaserParameters& laserParams);

    protected:
      std::vector<double> _ranges;
      std::vector<double> _remissions;
      LaserParameters _laserParams;
  };

} // end namespace

#endif
