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

#include <list>
#include <set>
#include <string>
#include <utility>

#include "g2o/config.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o_simulator_api.h"

namespace g2o {

class World;
class BaseSensor;

class G2O_SIMULATOR_API BaseWorldObject {
 public:
  explicit BaseWorldObject(World* world = nullptr) : world_(world) {}
  virtual ~BaseWorldObject() = default;
  void setWorld(World* world) { world_ = world; }
  World* world() { return world_; }
  OptimizableGraph* graph();
  std::shared_ptr<OptimizableGraph::Vertex> vertex() { return vertex_; }
  virtual void setVertex(
      const std::shared_ptr<OptimizableGraph::Vertex>& vertex);

 protected:
  World* world_;
  OptimizableGraph* graph_ = nullptr;
  std::shared_ptr<OptimizableGraph::Vertex> vertex_ = nullptr;
};

template <class VertexTypeT>
class WorldObject : public BaseWorldObject, VertexTypeT {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using VertexType = VertexTypeT;
  using EstimateType = typename VertexType::EstimateType;
  explicit WorldObject(World* world = nullptr) : BaseWorldObject(world) {
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
  BaseRobot(World* world, std::string name)
      : world_(world), name_(std::move(name)) {}
  void setWorld(World* world) { world_ = world; }
  World* world() { return world_; }
  const std::string& name() const { return name_; }
  OptimizableGraph* graph();
  bool addSensor(BaseSensor* sensor);
  const std::set<BaseSensor*>& sensors() { return sensors_; }
  virtual void sense();

 protected:
  World* world_;
  std::set<BaseSensor*> sensors_;
  std::string name_;
};

class G2O_SIMULATOR_API World {
 public:
  explicit World(OptimizableGraph* graph) : graph_(graph) {}
  OptimizableGraph* graph() { return graph_; }
  bool addRobot(BaseRobot* robot);
  bool addWorldObject(BaseWorldObject* worldObject);
  bool addParameter(const std::shared_ptr<Parameter>& p);

  std::set<BaseWorldObject*>& objects() { return objects_; }
  std::set<BaseRobot*>& robots() { return robots_; }

 protected:
  std::set<BaseWorldObject*> objects_;
  std::set<BaseRobot*> robots_;
  OptimizableGraph* graph_;
  int runningId_ = 0;
  int paramId_ = 0;
};

template <class RobotPoseObject>
class Robot : public BaseRobot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using PoseObject = RobotPoseObject;
  using TrajectoryType = std::list<PoseObject*>;
  using VertexType = typename PoseObject::VertexType;
  using PoseType = typename PoseObject::EstimateType;

  Robot(World* world, const std::string& name) : BaseRobot(world, name) {}
  virtual void relativeMove(const PoseType& movement_) {
    pose_ = pose_ * movement_;
    move(pose_);
  }

  virtual void move(const PoseType& pose) {
    pose_ = pose;
    if (world()) {
      auto* po = new PoseObject();
      po->vertex()->setEstimate(pose_);
      world()->addWorldObject(po);
      trajectory_.push_back(po);
    }
  }

  TrajectoryType& trajectory() { return trajectory_; }
  const PoseType& pose() const { return pose_; }

 protected:
  TrajectoryType trajectory_;
  PoseType pose_;
};

class G2O_SIMULATOR_API BaseSensor {
 public:
  explicit BaseSensor(std::string name) : name_(std::move(name)) {}
  inline BaseRobot* robot() { return robot_; }
  inline void setRobot(BaseRobot* robot) { robot_ = robot; }
  World* world();
  OptimizableGraph* graph();
  std::vector<Parameter*> parameters() { return parameters_; }
  virtual void sense() = 0;
  virtual void addParameters() {}

 protected:
  std::string name_;
  std::vector<Parameter*> parameters_;
  BaseRobot* robot_;
};

template <class RobotTypeT, class EdgeTypeT>
class UnarySensor : public BaseSensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using RobotType = RobotTypeT;
  using PoseObject = typename RobotTypeT::PoseObject;
  using TrajectoryType = typename RobotTypeT::TrajectoryType;
  using PoseVertexType = typename RobotTypeT::PoseObject::VertexType;
  using EdgeType = EdgeTypeT;
  using InformationType = typename EdgeTypeT::InformationType;

  explicit UnarySensor(const std::string& name) : BaseSensor(name) {
    information_.setIdentity();
  }

  void setInformation(const InformationType& information) {
    information_ = information;
    sampler_.setDistribution(information_.inverse());
  }

  const InformationType& information() { return information_; }

  void sense() override {
    robotPoseObject_ = nullptr;
    // set the robot pose
    if (!robot()) return;

    auto* r = dynamic_cast<RobotType*>(robot());
    if (!r) return;

    if (!r->trajectory().empty())
      robotPoseObject_ = *(r->trajectory().rbegin());

    if (!world() || !graph()) return;

    auto e = mkEdge();
    if (e) {
      e->setMeasurementFromState();
      addNoise(e.get());
      graph()->addEdge(e);
    }
  }

 protected:
  PoseObject* robotPoseObject_;
  InformationType information_;

  std::shared_ptr<EdgeType> mkEdge() {
    auto e = std::make_shared<EdgeType>();
    e->vertices()[0] = robotPoseObject_->vertex();
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> sampler_;
  virtual void addNoise(EdgeType*){};
};

template <class RobotTypeT, class EdgeTypeT, class WorldObjectTypeT>
class BinarySensor : public BaseSensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using RobotType = RobotTypeT;
  using PoseObject = typename RobotType::PoseObject;
  using TrajectoryType = typename RobotType::TrajectoryType;
  using PoseVertexType = typename RobotType::PoseObject::VertexType;
  using EdgeType = EdgeTypeT;
  using WorldObjectType = WorldObjectTypeT;
  using VertexType = typename WorldObjectType::VertexType;
  using InformationType = typename EdgeType::InformationType;

  explicit BinarySensor(const std::string& name) : BaseSensor(name) {
    information_.setIdentity();
  }

  void setInformation(const InformationType& information) {
    information_ = information;
    sampler_.setDistribution(information_.inverse());
  }

  const InformationType& information() { return information_; }

  void sense() override {
    robotPoseObject_ = nullptr;
    // set the robot pose
    if (!robot()) return;

    auto* r = dynamic_cast<RobotType*>(robot());
    if (!r) return;

    if (!r->trajectory().empty())
      robotPoseObject_ = *(r->trajectory().rbegin());

    if (!world() || !graph()) return;

    // naive search. just for initial testing
    for (auto it = world()->objects().begin(); it != world()->objects().end();
         ++it) {
      auto* wo = dynamic_cast<WorldObjectType*>(*it);
      if (wo) {
        auto e = mkEdge(wo);
        if (e) {
          e->setMeasurementFromState();
          addNoise(e.get());
          graph()->addEdge(e);
        }
      }
    }
  }

 protected:
  PoseObject* robotPoseObject_;
  InformationType information_;

  std::shared_ptr<EdgeType> mkEdge(WorldObjectType* object) {
    std::shared_ptr<EdgeType> e = std::make_shared<EdgeType>();
    e->vertices()[0] = robotPoseObject_->vertex();
    e->vertices()[1] = object->vertex();
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> sampler_;
  virtual void addNoise(EdgeType*){};
};

}  // namespace g2o

#endif
