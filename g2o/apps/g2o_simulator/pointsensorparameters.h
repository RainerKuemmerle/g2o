// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#ifndef G2O_POINTSENSORPARAMETERS_H_
#define G2O_POINTSENSORPARAMETERS_H_

#include <cmath>

#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API PointSensorParameters{
  public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointSensorParameters();
    double maxRange() const {return sqrt(_maxRange2);}
    void setMaxRange(double maxRange_)  {_maxRange2 = maxRange_*maxRange_;}
    double minRange() const {return sqrt(_minRange2);}
    void setMinRange(double minRange_)  {_minRange2 = minRange_*minRange_;}
    double fov() const {return _fov;}
    void setFov(double fov_)  {_fov = fov_;}
    double maxAngularDifference() const { return _maxAngularDifference; }
    void setMaxAngularDifference(double angularDifference) {_maxAngularDifference = angularDifference;}
  protected:
    double _maxRange2, _minRange2, _fov,  _maxAngularDifference;
  };

}

#endif
