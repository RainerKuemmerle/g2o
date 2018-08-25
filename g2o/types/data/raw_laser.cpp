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

#include "raw_laser.h"

#include <iostream>

using namespace std;

namespace g2o {

  RawLaser::RawLaser() :
    RobotData(),
    _laserParams(0, 180, -const_pi()/2, const_pi()/180, 50, cst(0.1), 0)
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
    number_t angle, fov, res, maxrange, acc;
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

  void RawLaser::setRanges(const vector<number_t>& ranges)
  {
    _ranges = ranges;
  }

  void RawLaser::setRemissions(const std::vector<number_t>& remissions)
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
      const number_t& r = _ranges[i];
      if (r < _laserParams.maxRange && r > _laserParams.minRange) {
        number_t alpha = _laserParams.firstBeamAngle + i * _laserParams.angularStep;
        points.push_back(Vector2(std::cos(alpha) * r, std::sin(alpha) * r));
      }
    }
    return points;
  }

} // end namespace
