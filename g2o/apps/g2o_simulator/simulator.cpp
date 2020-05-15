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

#include "simulator.h"
#include <iostream>
namespace g2o{
  using namespace std;

  // BaseWorldObject
  BaseWorldObject::~BaseWorldObject() {}
  OptimizableGraph* BaseWorldObject::graph() {
    if (_world)
      return _world-> graph();
    return 0;
  }

  void BaseWorldObject::setVertex(OptimizableGraph::Vertex* vertex_){ 
    _vertex= vertex_; 
  }

  // BaseRobot
  OptimizableGraph* BaseRobot::graph() {
    if (_world)
      return _world-> graph();
    return 0;
  }

  bool BaseRobot::addSensor(BaseSensor* sensor){
    assert (graph());
    std::pair<std::set<BaseSensor*>::iterator, bool> result=_sensors.insert(sensor);
    if(result.second){
      sensor->setRobot(this);
      sensor->addParameters();
    }
    return result.second;
  }

  void BaseRobot::sense() {
    for (std::set<BaseSensor*>::iterator it=_sensors.begin(); it!=_sensors.end(); ++it){
      BaseSensor* s=*it;
      s->sense();
    }
  }

  // Sensor
  World* BaseSensor::world() {
    if (!_robot)
      return 0;
    return _robot->world();
  }

  OptimizableGraph* BaseSensor::graph() {
    if (!_robot)
      return 0;
    return _robot->graph();
  }

  //World
  bool World::addRobot(BaseRobot* robot){
    std::pair<std::set<BaseRobot*>::iterator, bool> result=_robots.insert(robot);
    if (result.second){
      robot->setWorld(this);
    }
    return result.second;
  }
  
  bool World::addWorldObject(BaseWorldObject* object){
    std::pair<std::set<BaseWorldObject*>::iterator, bool> result=_objects.insert(object);
    if (result.second){
      object->setWorld(this);
    }
    if (graph() && object->vertex()){
      object->vertex()->setId(_runningId++);
      graph()->addVertex(object->vertex());
    }
    return result.second;
  }

  bool World::addParameter(Parameter* param){
    if ( !graph())
      return false;
    param->setId(_paramId);
    graph()->addParameter(param);
    _paramId++;
    return true;
  }


} // end namespace
