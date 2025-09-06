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

#include <memory>
#include <unordered_set>

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/optimizable_graph.h"

namespace g2o {

namespace {
class SimulationEstimatePropagatorCost : public EstimatePropagatorCostBase {
 public:
  double operator()(OptimizableGraph::Edge* edge,
                    const OptimizableGraph::VertexSet& from,
                    OptimizableGraph::Vertex* to) const override {
    return edge->initialEstimatePossible(from, to);
  }
  [[nodiscard]] std::string_view name() const override {
    return "simulation spanning tree";
  }
};
}  // namespace

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

void Simulator::finalize() {
  auto vertex_id_edge_lookup = world_.graph().createVertexEdgeLookup();

  // Drop vertices without any edge
  for (auto iter = world_.graph().vertices().begin();
       iter != world_.graph().vertices().end();) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(iter->second.get());
    auto v_edges = vertex_id_edge_lookup.lookup(v->id());
    if (v_edges.second && !v_edges.first->second.empty()) {
      ++iter;
      continue;
    }
    iter = world_.graph().vertices().erase(iter);
  }
  // Drop parameters without any edge
  std::unordered_set<int> connected_parameters;
  for (const auto& edge_ptr : world_.graph().edges()) {
    auto* edge = static_cast<OptimizableGraph::Edge*>(edge_ptr.get());
    for (const auto& p_id : edge->parameterIds()) {
      connected_parameters.insert(p_id);
    }
  }
  for (auto iter = world_.graph().parameters().begin();
       iter != world_.graph().parameters().end();) {
    if (connected_parameters.count(iter->first) > 0) {
      ++iter;
      continue;
    }
    iter = world_.graph().parameters().erase(iter);
  }

  // Initial estimate
  OptimizableGraph::VertexSet fixedVertices;
  for (const auto& id_v : world_.graph().vertices()) {
    auto v = std::dynamic_pointer_cast<OptimizableGraph::Vertex>(id_v.second);
    if (!v || !v->fixed()) continue;
    fixedVertices.emplace(std::move(v));
  }

  SimulationEstimatePropagatorCost propagator;
  EstimatePropagator estimatePropagator(&world_.graph());
  estimatePropagator.propagate(fixedVertices, propagator);
}

}  // namespace g2o
