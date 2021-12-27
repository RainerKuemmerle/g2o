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
namespace g2o {

// BaseWorldObject
OptimizableGraph* BaseWorldObject::graph() {
  if (world_) return world_->graph();
  return nullptr;
}

void BaseWorldObject::setVertex(
    const std::shared_ptr<OptimizableGraph::Vertex>& vertex) {
  vertex_ = vertex;
}

// BaseRobot
OptimizableGraph* BaseRobot::graph() {
  if (world_) return world_->graph();
  return nullptr;
}

bool BaseRobot::addSensor(BaseSensor* sensor) {
  assert(graph());
  std::pair<std::set<BaseSensor*>::iterator, bool> result =
      sensors_.insert(sensor);
  if (result.second) {
    sensor->setRobot(this);
    sensor->addParameters();
  }
  return result.second;
}

void BaseRobot::sense() {
  for (auto* s : sensors_) {
    s->sense();
  }
}

// Sensor
World* BaseSensor::world() {
  if (!robot_) return nullptr;
  return robot_->world();
}

OptimizableGraph* BaseSensor::graph() {
  if (!robot_) return nullptr;
  return robot_->graph();
}

// World
bool World::addRobot(BaseRobot* robot) {
  std::pair<std::set<BaseRobot*>::iterator, bool> result =
      robots_.insert(robot);
  if (result.second) {
    robot->setWorld(this);
  }
  return result.second;
}

bool World::addWorldObject(BaseWorldObject* object) {
  std::pair<std::set<BaseWorldObject*>::iterator, bool> result =
      objects_.insert(object);
  if (result.second) {
    object->setWorld(this);
  }
  if ((graph() != nullptr) && object->vertex()) {
    object->vertex()->setId(runningId_++);
    graph()->addVertex(object->vertex());
  }
  return result.second;
}

bool World::addParameter(const std::shared_ptr<Parameter>& param) {
  if (!graph()) return false;
  param->setId(paramId_);
  graph()->addParameter(param);
  paramId_++;
  return true;
}

}  // namespace g2o
