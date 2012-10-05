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

#include "edge_se2_pure_calib.h"

namespace g2o {

EdgeSE2PureCalib::EdgeSE2PureCalib()
{
}

bool EdgeSE2PureCalib::read(std::istream& is)
{
  (void) is;
  return false;
}

bool EdgeSE2PureCalib::write(std::ostream& os) const
{
  (void) os;
  return false;
}

void EdgeSE2PureCalib::computeError()
{
  const VertexSE2* laserOffset = static_cast<const VertexSE2*>(_vertices[0]);
  const VertexOdomDifferentialParams* odomParams = dynamic_cast<const VertexOdomDifferentialParams*>(_vertices[1]);

  // get the calibrated motion given by the odometry
  VelocityMeasurement calibratedVelocityMeasurment(measurement().velocityMeasurement.vl() * odomParams->estimate()(0),
      measurement().velocityMeasurement.vr() * odomParams->estimate()(1),
      measurement().velocityMeasurement.dt());
  MotionMeasurement mm = OdomConvert::convertToMotion(calibratedVelocityMeasurment, odomParams->estimate()(2));
  SE2 Ku_ij;
  Ku_ij.fromVector(mm.measurement());

  SE2 laserMotionInRobotFrame = laserOffset->estimate() * measurement().laserMotion * laserOffset->estimate().inverse();
  SE2 delta = Ku_ij.inverse() * laserMotionInRobotFrame;
  _error = delta.toVector();
}

} // end namespace
