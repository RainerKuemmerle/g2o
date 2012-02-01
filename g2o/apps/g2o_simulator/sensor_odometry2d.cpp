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

#include <cassert>
#include <iostream>
#include "sensor_odometry2d.h"

// Robot2D
namespace g2o {
  using namespace std;


  SensorOdometry2D::SensorOdometry2D(const std::string& name_):
    BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_){}

  void SensorOdometry2D::sense(){
    
    if (! robot())
      return;
    
    RobotType* r =dynamic_cast<RobotType*>(robot());
    if (!r)
      return;
    
    PoseObject* pprev=0, *pcurr=0;
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    if (it!=r->trajectory().rend()){
      pcurr = *it; 
      it++;
    }
    if (it!=r->trajectory().rend()){
      pprev = *it; 
      it++;
    }
    if (!(pcurr&&pprev)) {
      cerr << __PRETTY_FUNCTION__ << ": fatal, trajectory empty" << endl;
      return;
    }
    _robotPoseObject = pprev;
    EdgeType* e=mkEdge(pcurr);
    if (e){
      e->setMeasurementFromState();
      addNoise(e);
      if (graph())
    graph()->addEdge(e);
    }
    _robotPoseObject = pcurr;
  }
  
  void SensorOdometry2D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

}//
