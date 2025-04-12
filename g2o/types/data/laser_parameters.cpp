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

#include "laser_parameters.h"

namespace g2o {

LaserParameters::LaserParameters(int t, int nbeams, double _firstBeamAngle,
                                 double _angularStep, double _maxRange,
                                 double _accuracy, int _remissionMode,
                                 double _minRange)
    : laserPose(SE2(0., 0., 0.)),
      type(t),
      firstBeamAngle(_firstBeamAngle),
      fov(_angularStep * nbeams),
      angularStep(_angularStep),
      accuracy(_accuracy),
      remissionMode(_remissionMode),
      maxRange(_maxRange),
      minRange(_minRange) {}

LaserParameters::LaserParameters(int nbeams, double _firstBeamAngle,
                                 double _angularStep, double _maxRange,
                                 double _minRange)
    : laserPose(SE2(0., 0., 0.)),
      type(0),
      firstBeamAngle(_firstBeamAngle),
      fov(_angularStep * nbeams),
      angularStep(_angularStep),
      accuracy(0.1),
      remissionMode(0),
      maxRange(_maxRange),
      minRange(_minRange) {}

}  // namespace g2o
