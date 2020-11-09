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

#include "odometry_measurement.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {

  VelocityMeasurement::VelocityMeasurement() :
    _measurement(0., 0.), _dt(0.)
  {
  }

  VelocityMeasurement::VelocityMeasurement(number_t vl, number_t vr, number_t dt) :
    _measurement(vl, vr), _dt(dt)
  {
  }

  MotionMeasurement::MotionMeasurement() :
    _measurement(0., 0., 0.), _dt(0.)
  {
  }

  MotionMeasurement::MotionMeasurement(number_t x, number_t y, number_t theta, number_t dt) :
    _measurement(x, y, theta), _dt(dt)
  {
  }

  MotionMeasurement::MotionMeasurement(const Vector3& m, number_t dt) :
    _measurement(m), _dt(dt)
  {
  }

  VelocityMeasurement OdomConvert::convertToVelocity(const MotionMeasurement& m)
  {
    if (fabs(m.theta()) > 1e-7) {
      const number_t translation = std::hypot(m.x(), m.y());
      const number_t R = translation / (2 * sin(m.theta() / 2));
      number_t w = 0.;
      if (fabs(m.dt()) > 1e-7) w = m.theta() / m.dt();

      const number_t vl = (2.*R*w - w) / 2.;
      const number_t vr = w + vl;

      return VelocityMeasurement(vl, vr, m.dt());
    } else {
      number_t vl, vr;
      if (fabs(m.dt()) > 1e-7)
        vl = vr = std::hypot(m.x(), m.y()) / m.dt();
      else
        vl = vr = 0.;
      return VelocityMeasurement(vl, vr, m.dt());
    }
  }

  MotionMeasurement OdomConvert::convertToMotion(const VelocityMeasurement& v, number_t l)
  {
    number_t x, y, theta;
    if (fabs(v.vr() - v.vl()) > 1e-7) {
      number_t R = l * 0.5 * ((v.vl() + v.vr())  / (v.vr() - v.vl()));
      number_t w = (v.vr() - v.vl()) / l;

      theta = w * v.dt();
      Rotation2D rot(theta);
      Vector2 icc(0, R);
      Vector2 motion = (rot * (Vector2(-1.*icc))) + icc;
      x = motion.x();
      y = motion.y();
    } else {
      number_t tv = 0.5 * (v.vr() + v.vl());
      theta = 0.;
      x = tv * v.dt();
      y = 0.;
    }

    return MotionMeasurement(x, y, theta, v.dt());
  }

} // end namespace
