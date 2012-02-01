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

#ifndef G2O_SENSOR_POSE3D_OFFSET_H_
#define G2O_SENSOR_POSE3D_OFFSET_H_
#include "simulator3d_base.h"
#include "pointsensorparameters.h"

namespace g2o {

  class SensorPose3DOffset : public PointSensorParameters, public BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3>  { 
  public:
    SensorPose3DOffset(const std::string& name_);
    virtual void sense();
    int stepsToIgnore() const {return _stepsToIgnore;}
    void setStepsToIgnore(int stepsToIgnore_) {_stepsToIgnore = stepsToIgnore_;}
    void addNoise(EdgeType* e);
    virtual void addParameters();
    ParameterSE3Offset* offsetParam1() {return _offsetParam1;};
    ParameterSE3Offset* offsetParam2() {return _offsetParam2;};

  protected:
    
    bool isVisible(WorldObjectType* to);
    int _stepsToIgnore;
    ParameterSE3Offset* _offsetParam1, *_offsetParam2;

    // these are temporaries
    std::set<PoseObject*> _posesToIgnore;
  };

}

#endif
