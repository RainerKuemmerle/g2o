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

namespace g2o {

RawLaser::RawLaser()
    : laserParams_(0, 180, -const_pi() / 2, const_pi() / 180, 50, cst(0.1), 0) {
}

bool RawLaser::write(std::ostream& /*os*/) const {
  // TODO(goki):
  std::cerr << "RawLaser::write() not implemented yet." << std::endl;
  return false;
}

bool RawLaser::read(std::istream& is) {
  int type;
  number_t angle;
  number_t fov;
  number_t res;
  number_t maxrange;
  number_t acc;
  int remission_mode;
  is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;

  int beams;
  is >> beams;
  laserParams_ =
      LaserParameters(type, beams, angle, res, maxrange, acc, remission_mode);
  ranges_.resize(beams);
  for (int i = 0; i < beams; i++) is >> ranges_[i];

  is >> beams;
  remissions_.resize(beams);
  for (int i = 0; i < beams; i++) is >> remissions_[i];

  // timestamp + host
  is >> timestamp_;
  is >> hostname_;
  is >> loggerTimestamp_;
  return true;
}

void RawLaser::setRanges(const std::vector<number_t>& ranges) {
  ranges_ = ranges;
}

void RawLaser::setRemissions(const std::vector<number_t>& remissions) {
  remissions_ = remissions;
}

void RawLaser::setLaserParams(const LaserParameters& laserParams) {
  laserParams_ = laserParams;
}

RawLaser::Point2DVector RawLaser::cartesian() const {
  Point2DVector points;
  for (size_t i = 0; i < ranges_.size(); ++i) {
    const number_t& r = ranges_[i];
    if (r < laserParams_.maxRange && r > laserParams_.minRange) {
      const number_t alpha =
          laserParams_.firstBeamAngle + i * laserParams_.angularStep;
      points.push_back(Vector2(std::cos(alpha) * r, std::sin(alpha) * r));
    }
  }
  return points;
}

}  // namespace g2o
