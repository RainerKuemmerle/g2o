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

  VelocityMeasurement::VelocityMeasurement(double vl, double vr, double dt) :
    _measurement(vl, vr), _dt(dt)
  {
  }

  MotionMeasurement::MotionMeasurement() :
    _measurement(0., 0., 0.), _dt(0.)
  {
  }

  MotionMeasurement::MotionMeasurement(double x, double y, double theta, double dt) :
    _measurement(x, y, theta), _dt(dt)
  {
  }

  MotionMeasurement::MotionMeasurement(const Eigen::Vector3d& m, double dt) :
    _measurement(m), _dt(dt)
  {
  }

  VelocityMeasurement OdomConvert::convertToVelocity(const MotionMeasurement& m)
  {
    Eigen::Vector2d px2(0, 10);

    if (fabs(m.theta()) > 1e-7) {
      Eigen::Rotation2Dd rot(m.theta());
      Eigen::Vector2d px3(m.x(), m.y());
      Eigen::Vector2d px4(rot * px2 + px3);

      const double& y2 = px2.y();
      const double& x3 = px3.x();
      const double& y3 = px3.y();
      const double& x4 = px4.x();
      const double& y4 = px4.y();

      double R = (y2 * (x3*y4 - y3*x4)) / (y2 * (x3 - x4));
      double w;
      if (fabs(m.dt()) > 1e-7)
        w = m.theta() / m.dt();
      else
        w = 0.;

      double vl = (2.*R*w - w) / 2.;
      double vr = w + vl;

      return VelocityMeasurement(vl, vr, m.dt());
    } else {
      double vl, vr;
      if (fabs(m.dt()) > 1e-7)
        vl = vr = hypot(m.x(), m.y()) / m.dt();
      else
        vl = vr = 0.;
      return VelocityMeasurement(vl, vr, m.dt());
    }
  }

  MotionMeasurement OdomConvert::convertToMotion(const VelocityMeasurement& v, double l)
  {
    double x, y, theta;
    if (fabs(v.vr() - v.vl()) > 1e-7) {
      double R = l * 0.5 * ((v.vl() + v.vr())  / (v.vr() - v.vl()));
      double w = (v.vr() - v.vl()) / l;

      theta = w * v.dt();
      Eigen::Rotation2Dd rot(theta);
      Eigen::Vector2d icc(0, R);
      Eigen::Vector2d motion = (rot * (Eigen::Vector2d(-1.*icc))) + icc;
      x = motion.x();
      y = motion.y();
    } else {
      double tv = 0.5 * (v.vr() + v.vl());
      theta = 0.;
      x = tv * v.dt();
      y = 0.;
    }

    return MotionMeasurement(x, y, theta, v.dt());

  }

} // end namespace
