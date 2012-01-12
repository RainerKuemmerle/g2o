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

#include "laser_parameters.h"

namespace g2o {

  LaserParameters::LaserParameters(int t, int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange, double _accuracy, int _remissionMode)
  {
    type           = t;
    firstBeamAngle = _firstBeamAngle;
    angularStep    = _angularStep;
    maxRange       = _maxRange;
    laserPose      = SE2(0., 0., 0.);
    maxRange       = maxRange;
    accuracy       = _accuracy;
    remissionMode  = _remissionMode;
    fov            = angularStep * nbeams;
  }

  LaserParameters::LaserParameters(int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange)
  {
    type           = 0;
    firstBeamAngle = _firstBeamAngle;
    angularStep    = _angularStep;
    maxRange       = _maxRange;
    laserPose      = SE2(0., 0., 0.);
    maxRange       = maxRange;
    accuracy       = 0.1;
    remissionMode  = 0;
    fov            = angularStep * nbeams;
  }

} // end namespace
