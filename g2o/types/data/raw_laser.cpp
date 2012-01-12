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

#include "raw_laser.h"

#include <iostream>

using namespace std;

namespace g2o {

  RawLaser::RawLaser() :
    RobotData(),
    _laserParams(0, 180, -M_PI/2, M_PI/180., 50.,0.1, 0)
  {
  }

  RawLaser::~RawLaser()
  {
  }

  bool RawLaser::write(std::ostream& /*os*/) const
  {
    // TODO
    cerr << "RawLaser::write() not implemented yet." << endl;
    return false;
  }

  bool RawLaser::read(std::istream& is)
  {
    int type;
    double angle, fov, res, maxrange, acc;
    int remission_mode;
    is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;

    int beams;
    is >> beams;
    _laserParams = LaserParameters(type, beams, angle, res, maxrange, acc, remission_mode);      
    _ranges.resize(beams);
    for (int i=0; i<beams; i++)
      is >> _ranges[i];

    is >> beams;
    _remissions.resize(beams);
    for (int i=0; i < beams; i++)
      is >> _remissions[i];

    // timestamp + host
    is >> _timestamp;
    is >> _hostname;
    is >> _loggerTimestamp;
    return true;
  }

  void RawLaser::setRanges(const vector<double>& ranges)
  {
    _ranges = ranges;
  }

  void RawLaser::setRemissions(const std::vector<double>& remissions)
  {
    _remissions = remissions;
  }

  void RawLaser::setLaserParams(const LaserParameters& laserParams)
  {
    _laserParams = laserParams;
  }

  RawLaser::Point2DVector RawLaser::cartesian() const
  {
    Point2DVector points;
    for (size_t i = 0; i < _ranges.size(); ++i) {
      const double& r = _ranges[i];
      if (r < _laserParams.maxRange) {
        double alpha = _laserParams.firstBeamAngle + i * _laserParams.angularStep;
        points.push_back(Eigen::Vector2d(cos(alpha) * r, sin(alpha) * r));
      }
    }
    return points;
  }

} // end namespace
