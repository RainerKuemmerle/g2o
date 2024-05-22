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

#ifndef G2O_SIMULATOR_
#define G2O_SIMULATOR_

#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/sampler.h"
#include "g2o_simulator_api.h"

namespace g2o {

class World;
class BaseSensor;

class G2O_SIMULATOR_API BaseWorldObject {
 public:
  BaseWorldObject() = default;
  virtual ~BaseWorldObject() = default;
  std::shared_ptr<OptimizableGraph::Vertex> vertex() { return vertex_; }
  virtual void setVertex(
      const std::shared_ptr<OptimizableGraph::Vertex>& vertex);

 protected:
  std::shared_ptr<OptimizableGraph::Vertex> vertex_ = nullptr;
};

template <class VertexTypeT>
class WorldObject : public BaseWorldObject, VertexTypeT {
 public:
  using VertexType = VertexTypeT;
  using EstimateType = typename VertexType::EstimateType;
  explicit WorldObject() : BaseWorldObject() {
    vertex_ = std::make_shared<VertexType>();
  }
  void setVertex(
      const std::shared_ptr<OptimizableGraph::Vertex>& vertex) override {
    if (!dynamic_cast<VertexType*>(vertex.get())) return;
    vertex_ = vertex;
  }

  std::shared_ptr<VertexType> vertex() {
    if (!vertex_) return nullptr;
    return std::dynamic_pointer_cast<VertexType>(vertex_);
  }
};

class G2O_SIMULATOR_API BaseRobot {
 public:
  explicit BaseRobot(std::string name) : name_(std::move(name)) {}
  virtual ~BaseRobot() = default;

  BaseRobot(BaseRobot const&) = delete;
  BaseRobot& operator=(BaseRobot const&) = delete;

  [[nodiscard]] const std::string& name() const { return name_; }
  void addSensor(std::unique_ptr<BaseSensor>, World& world);
  const std::vector<std::unique_ptr<BaseSensor>>& sensors() { return sensors_; }
  virtual void sense(World& world);

  [[nodiscard]] const std::vector<int>& trajectory() const {
    return trajectory_;
  }

 protected:
  std::vector<std::unique_ptr<BaseSensor>> sensors_;
  std::string name_;
  std::vector<int> trajectory_;
};

class G2O_SIMULATOR_API World {
 public:
  World() = default;
  World(World const&) = delete;
  World& operator=(World const&) = delete;

  OptimizableGraph& graph() { return graph_; }
  const OptimizableGraph& graph() const { return graph_; }
  void addRobot(std::unique_ptr<BaseRobot> robot);
  //! returns the ID of the added object
  int addWorldObject(std::unique_ptr<BaseWorldObject> worldObject);
  bool addParameter(const std::shared_ptr<Parameter>& p);

  [[nodiscard]] const std::vector<std::unique_ptr<BaseWorldObject>>& objects()
      const {
    return objects_;
  }
  [[nodiscard]] const std::vector<std::unique_ptr<BaseRobot>>& robots() const {
    return robots_;
  }

 protected:
  std::vector<std::unique_ptr<BaseWorldObject>> objects_;
  std::vector<std::unique_ptr<BaseRobot>> robots_;
  OptimizableGraph graph_;
  int runningId_ = 0;
  int paramId_ = 0;
};

template <class RobotPoseObject>
class Robot : public BaseRobot {
 public:
  using PoseObject = RobotPoseObject;
  using VertexType = typename PoseObject::VertexType;
  using PoseType = typename PoseObject::EstimateType;

  explicit Robot(std::string name) : BaseRobot(name) {}
  virtual void relativeMove(World& world, const PoseType& movement_) {
    move(world, pose_ * movement_);
  }

  virtual void move(World& world, const PoseType& pose) {
    pose_ = pose;
    auto po = std::make_unique<PoseObject>();
    po->vertex()->setEstimate(pose_);
    const int pose_id = world.addWorldObject(std::move(po));
    trajectory_.emplace_back(pose_id);
  }

  const PoseType& pose() const { return pose_; }

 protected:
  PoseType pose_;
};

class G2O_SIMULATOR_API BaseSensor {
 public:
  explicit BaseSensor(std::string name) : name_(std::move(name)) {}
  virtual ~BaseSensor() = default;

  virtual void addParameters(World& /*world*/) {}
  [[nodiscard]] const std::vector<Parameter*>& parameters() const {
    return parameters_;
  }

  virtual void sense(BaseRobot& robot, World& world) = 0;

  template <typename T>
  std::shared_ptr<T> robotPoseVertex(BaseRobot& robot, World& world) {
    if (robot.trajectory().empty()) return nullptr;
    return robotPoseVertexForId<T>(robot.trajectory().back(), world);
  }

  template <typename T>
  std::shared_ptr<T> robotPoseVertexForId(int id, World& world) {
    return std::dynamic_pointer_cast<T>(world.graph().vertex(id));
  }

 protected:
  std::string name_;
  std::vector<Parameter*> parameters_;
};

template <class RobotTypeT, class EdgeTypeT>
class UnarySensor : public BaseSensor {
 public:
  using RobotType = RobotTypeT;
  using PoseObject = typename RobotTypeT::PoseObject;
  using PoseVertexType = typename RobotTypeT::PoseObject::VertexType;
  using EdgeType = EdgeTypeT;
  using InformationType = typename EdgeTypeT::InformationType;

  explicit UnarySensor(std::string name) : BaseSensor(std::move(name)) {
    sampler_.setDistribution(information_.inverse());
  }

  void setInformation(const InformationType& information) {
    information_ = information;
    sampler_.setDistribution(information_.inverse());
  }

  const InformationType& information() { return information_; }

  void sense(BaseRobot& robot, World& world) override {
    // set the robot pose
    auto* r = dynamic_cast<RobotType*>(&robot);
    if (!r) return;

    robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);

    auto e = mkEdge();
    if (e) {
      e->setMeasurementFromState();
      addNoise(e.get());
      world.graph().addEdge(e);
    }
  }

 protected:
  std::shared_ptr<PoseVertexType> robotPoseVertex_;
  InformationType information_ = InformationType::Identity();

  std::shared_ptr<EdgeType> mkEdge() {
    auto e = std::make_shared<EdgeType>();
    e->vertices()[0] = robotPoseVertex_;
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> sampler_;
  virtual void addNoise(EdgeType*) {};
};

template <class RobotTypeT, class EdgeTypeT, class WorldObjectTypeT>
class BinarySensor : public BaseSensor {
 public:
  using RobotType = RobotTypeT;
  using PoseObject = typename RobotType::PoseObject;
  using PoseVertexType = typename RobotType::PoseObject::VertexType;
  using EdgeType = EdgeTypeT;
  using WorldObjectType = WorldObjectTypeT;
  using VertexType = typename WorldObjectType::VertexType;
  using InformationType = typename EdgeType::InformationType;

  explicit BinarySensor(std::string name) : BaseSensor(std::move(name)) {
    sampler_.setDistribution(information_.inverse());
  }

  void setInformation(const InformationType& information) {
    information_ = information;
    sampler_.setDistribution(information_.inverse());
  }

  const InformationType& information() { return information_; }

  void sense(BaseRobot& robot, World& world) override {
    auto* r = dynamic_cast<RobotType*>(&robot);
    if (!r) return;

    robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);

    // naive search. just for initial testing
    for (const auto& base_world_object : world.objects()) {
      auto* wo = dynamic_cast<WorldObjectType*>(base_world_object.get());
      if (!wo) continue;
      auto e = mkEdge(wo);
      if (!e) continue;
      e->setMeasurementFromState();
      addNoise(e.get());
      world.graph().addEdge(e);
    }
  }

 protected:
  std::shared_ptr<PoseVertexType> robotPoseVertex_ = nullptr;
  InformationType information_ = InformationType::Identity();

  std::shared_ptr<EdgeType> mkEdge(WorldObjectType* object) {
    std::shared_ptr<EdgeType> e = std::make_shared<EdgeType>();
    e->vertices()[0] = robotPoseVertex_;
    e->vertices()[1] = object ? object->vertex() : nullptr;
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> sampler_;
  virtual void addNoise(EdgeType*) {};
};

/**
 * @brief Base class for the simulator engine.
 */
class Simulator {
 public:
  virtual ~Simulator() = default;
  virtual void setup() = 0;
  virtual void simulate() = 0;

  //! seed the random number generator
  void seed(std::mt19937::result_type value = std::mt19937::default_seed) {
    generator_.seed(value);
  }

  const g2o::World& world() const { return world_; }
  g2o::World& world() { return world_; }

 protected:
  std::mt19937 generator_;
  g2o::World world_;
};

}  // namespace g2o

#endif
