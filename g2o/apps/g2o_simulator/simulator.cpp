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

namespace g2o {

void BaseWorldObject::setVertex(
    const std::shared_ptr<OptimizableGraph::Vertex>& vertex) {
  vertex_ = vertex;
}

void BaseRobot::addSensor(std::unique_ptr<BaseSensor> sensor, World& world) {
  sensor->addParameters(world);
  sensors_.emplace_back(std::move(sensor));
}

void BaseRobot::sense(World& world) {
  for (const auto& s : sensors_) {
    s->sense(*this, world);
  }
}

// World
void World::addRobot(std::unique_ptr<BaseRobot> robot) {
  robots_.emplace_back(std::move(robot));
}

int World::addWorldObject(std::unique_ptr<BaseWorldObject> object) {
  if (!object->vertex()) return -1;
  object->vertex()->setId(runningId_++);
  graph().addVertex(object->vertex());
  const int id = object->vertex()->id();
  objects_.emplace_back(std::move(object));
  return id;
}

bool World::addParameter(const std::shared_ptr<Parameter>& param) {
  param->setId(paramId_);
  bool result = graph().addParameter(param);
  paramId_++;
  return result;
}

}  // namespace g2o
