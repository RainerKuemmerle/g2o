// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include <fstream>
#include <iostream>
#include <utility>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

namespace g2o {

static Isometry3 sample_noise_from_se3(const Vector6& cov) {
  double nx = g2o::Sampler::gaussRand(0., cov(0));
  double ny = g2o::Sampler::gaussRand(0., cov(1));
  double nz = g2o::Sampler::gaussRand(0., cov(2));

  double nroll = g2o::Sampler::gaussRand(0., cov(3));
  double npitch = g2o::Sampler::gaussRand(0., cov(4));
  double nyaw = g2o::Sampler::gaussRand(0., cov(5));

  AngleAxis aa(AngleAxis(nyaw, Vector3::UnitZ()) *
               AngleAxis(nroll, Vector3::UnitX()) *
               AngleAxis(npitch, Vector3::UnitY()));

  Isometry3 retval = Isometry3::Identity();
  retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
  retval.translation() = Vector3(nx, ny, nz);
  return retval;
}

static Vector3 sample_noise_from_plane(const Vector3& cov) {
  return Vector3(g2o::Sampler::gaussRand(0., cov(0)),
                 g2o::Sampler::gaussRand(0., cov(1)),
                 g2o::Sampler::gaussRand(0., cov(2)));
}

struct SimulatorItem {
  explicit SimulatorItem(OptimizableGraph* graph) : graph_(graph) {}
  OptimizableGraph* graph() { return graph_; }
  virtual ~SimulatorItem() = default;

 protected:
  OptimizableGraph* graph_;
};

struct WorldItem : public SimulatorItem {
  explicit WorldItem(
      OptimizableGraph* graph_,
      std::shared_ptr<OptimizableGraph::Vertex> vertex_ = nullptr)
      : SimulatorItem(graph_), vertex_(std::move(vertex_)) {}
  std::shared_ptr<OptimizableGraph::Vertex> vertex() { return vertex_; }
  void setVertex(const std::shared_ptr<OptimizableGraph::Vertex>& vertex) {
    vertex_ = vertex;
  }

 protected:
  std::shared_ptr<OptimizableGraph::Vertex> vertex_;
};

using WorldItemSet = std::set<WorldItem*>;

struct Robot;

struct Sensor {
  explicit Sensor(Robot* robot) : robot_(robot) {}
  Robot* robot() { return robot_; }
  virtual bool isVisible(const WorldItem*) const { return false; }
  virtual bool sense(WorldItem*, const Isometry3&) { return false; }
  virtual ~Sensor() = default;
  ;

 protected:
  Robot* robot_;
};

using SensorVector = std::vector<Sensor*>;

struct Robot : public WorldItem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit Robot(OptimizableGraph* graph_) : WorldItem(graph_) {
    planarMotion = false;
    position = Isometry3::Identity();
  }

  void move(const Isometry3& newPosition, int& id) {
    Isometry3 delta = position.inverse() * newPosition;
    position = newPosition;
    auto v = std::make_shared<VertexSE3>();
    v->setId(id);
    id++;
    graph()->addVertex(v);
    if (planarMotion) {
      // add a singleton constraint that locks the position of the robot on the
      // plane
      auto planeConstraint = std::make_shared<EdgeSE3Prior>();
      Matrix6 pinfo = Matrix6::Zero();
      pinfo(2, 2) = 1e9;
      planeConstraint->setInformation(pinfo);
      planeConstraint->setMeasurement(Isometry3::Identity());
      planeConstraint->vertices()[0] = v;
      planeConstraint->setParameterId(0, 0);
      graph()->addEdge(planeConstraint);
    }
    if (vertex()) {
      auto oldV = std::dynamic_pointer_cast<VertexSE3>(vertex());
      auto e = std::make_shared<EdgeSE3>();
      Isometry3 noise = sample_noise_from_se3(nmovecov);
      e->setMeasurement(delta * noise);
      Matrix6 m = Vector6(1. / nmovecov.array()).asDiagonal();
      e->setInformation(m);
      e->vertices()[0] = vertex();
      e->vertices()[1] = v;
      graph()->addEdge(e);
      v->setEstimate(oldV->estimate() * e->measurement());
    } else {
      v->setEstimate(position);
    }
    setVertex(v);
  }

  void relativeMove(const Isometry3& delta, int& id) {
    Isometry3 newPosition = position * delta;
    move(newPosition, id);
  }

  void sense(WorldItem* wi = nullptr) {
    for (auto* s : sensors) {
      s->sense(wi, position);
    }
  }

  Isometry3 position;
  SensorVector sensors;
  Vector6 nmovecov;
  bool planarMotion;
};

using RobotVector = std::vector<Robot*>;

struct Simulator : public SimulatorItem {
  explicit Simulator(OptimizableGraph* graph_) : SimulatorItem(graph_) {}
  void sense(int robotIndex) {
    Robot* r = robots[robotIndex];
    for (auto* item : world) {
      r->sense(item);
    }
  }

  void move(int robotIndex, const Isometry3& newRobotPose) {
    Robot* r = robots[robotIndex];
    r->move(newRobotPose, lastVertexId);
  }

  void relativeMove(int robotIndex, const Isometry3& delta) {
    Robot* r = robots[robotIndex];
    r->relativeMove(delta, lastVertexId);
  }

  int lastVertexId = 0;
  WorldItemSet world;
  RobotVector robots;
};

struct PlaneItem : public WorldItem {
  PlaneItem(OptimizableGraph* graph_, int id) : WorldItem(graph_) {
    auto p = std::make_shared<VertexPlane>();
    p->setId(id);
    graph()->addVertex(p);
    setVertex(p);
  }
};

struct PlaneSensor : public Sensor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PlaneSensor(Robot* r, int offsetId, const Isometry3& offset_) : Sensor(r) {
    _offsetVertex = std::make_shared<VertexSE3>();
    _offsetVertex->setId(offsetId);
    _offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(_offsetVertex);
  };

  bool isVisible(const WorldItem* wi) const override {
    if (!wi) return false;
    const auto* pi = dynamic_cast<const PlaneItem*>(wi);
    return pi != nullptr;
  }

  bool sense(WorldItem* wi, const Isometry3& position) override {
    if (!wi) return false;
    auto* pi = dynamic_cast<PlaneItem*>(wi);
    if (!pi) return false;
    auto rv = robot()->vertex();
    if (!rv) {
      return false;
    }
    auto robotVertex = std::dynamic_pointer_cast<VertexSE3>(rv);
    if (!robotVertex) {
      return false;
    }
    const Isometry3& robotPose = position;
    Isometry3 sensorPose = robotPose * _offsetVertex->estimate();
    auto planeVertex = std::dynamic_pointer_cast<VertexPlane>(pi->vertex());
    Plane3D worldPlane = planeVertex->estimate();

    Plane3D measuredPlane = sensorPose.inverse() * worldPlane;

    auto e = std::make_shared<EdgeSE3PlaneSensorCalib>();
    e->vertices()[0] = robotVertex;
    e->vertices()[1] = planeVertex;
    e->vertices()[2] = _offsetVertex;
    Vector3 noise = sample_noise_from_plane(_nplane);
    measuredPlane.oplus(noise);
    e->setMeasurement(measuredPlane);
    Matrix3 m = Matrix3::Zero();
    m(0, 0) = 1. / (_nplane(0));
    m(1, 1) = 1. / (_nplane(1));
    m(2, 2) = 1. / (_nplane(2));
    e->setInformation(m);
    robot()->graph()->addEdge(e);
    return true;
  }

  std::shared_ptr<VertexSE3> _offsetVertex;
  Vector3 _nplane;
};

static int simulator_3d_plane(int argc, char** argv) {
  CommandArgs arg;
  bool fixSensor;
  bool fixPlanes;
  bool fixFirstPose;
  bool fixTrajectory;
  bool planarMotion;
  std::cerr << "graph" << std::endl;
  arg.param("fixSensor", fixSensor, false,
            "fix the sensor position on the robot");
  arg.param("fixTrajectory", fixTrajectory, false, "fix the trajectory");
  arg.param("fixFirstPose", fixFirstPose, false, "fix the first robot pose");
  arg.param("fixPlanes", fixPlanes, false,
            "fix the planes (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.parseArgs(argc, argv);

  auto* g = new SparseOptimizer();
  auto odomOffset = std::make_shared<ParameterSE3Offset>();
  odomOffset->setId(0);
  g->addParameter(odomOffset);

  std::cerr << "sim" << std::endl;
  auto* sim = new Simulator(g);

  std::cerr << "robot" << std::endl;
  auto* r = new Robot(g);

  std::cerr << "planeSensor" << std::endl;
  Matrix3 R = Matrix3::Identity();
  R << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  Isometry3 sensorPose = Isometry3::Identity();
  sensorPose.matrix().block<3, 3>(0, 0) = R;
  sensorPose.translation() = Vector3(.3, 0.5, 1.2);
  auto* ps = new PlaneSensor(r, 0, sensorPose);
  ps->_nplane << 0.03, 0.03, 0.005;
  r->sensors.push_back(ps);
  sim->robots.push_back(r);

  std::cerr << "p1" << std::endl;
  Plane3D plane;
  auto* pi = new PlaneItem(g, 1);
  plane.fromVector(Eigen::Vector4d(0., 0., 1., 5.));
  static_cast<VertexPlane*>(pi->vertex().get())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->world.insert(pi);

  plane.fromVector(Eigen::Vector4d(1., 0., 0., 5.));
  pi = new PlaneItem(g, 2);
  static_cast<VertexPlane*>(pi->vertex().get())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->world.insert(pi);

  std::cerr << "p2" << std::endl;
  pi = new PlaneItem(g, 3);
  plane.fromVector(Eigen::Vector4d(0., 1., 0., 5.));
  static_cast<VertexPlane*>(pi->vertex().get())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->world.insert(pi);

  Quaternion q;
  Quaternion iq;
  if (planarMotion) {
    r->planarMotion = true;
    r->nmovecov << 0.01, 0.0025, 1e-9, 0.001, 0.001, 0.025;
    q = Quaternion(AngleAxis(0.2, Vector3::UnitZ()));
    iq = Quaternion(AngleAxis(-0.2, Vector3::UnitZ()));
  } else {
    r->planarMotion = false;
    // r->nmovecov << 0.1, 0.005, 1e-9, 0.05, 0.001, 0.001;
    r->nmovecov << 0.1, 0.005, 1e-9, 0.001, 0.001, 0.05;
    q = Quaternion(AngleAxis(M_PI / 10, Vector3::UnitZ()) *
                   AngleAxis(0.1, Vector3::UnitY()));
    iq = Quaternion(AngleAxis(-M_PI / 10, Vector3::UnitZ()) *
                    AngleAxis(0.1, Vector3::UnitY()));
  }

  sim->lastVertexId = 4;

  Isometry3 startPose(
      AngleAxis(-0.75 * M_PI, Vector3::UnitZ()).toRotationMatrix());
  sim->move(0, startPose);

  int k = 20;
  int l = 2;
  double delta_t = 0.2;
  for (int j = 0; j < l; j++) {
    Isometry3 delta(j != (l - 1) ? q.toRotationMatrix() : Matrix3::Identity());
    delta.translation() = Vector3(delta_t * j, 0., 0.);
    Isometry3 iDelta = delta.inverse();
    for (int a = 0; a < 2; a++) {
      for (int i = 0; i < k; i++) {
        std::cerr << "m";
        if (a == 0)
          sim->relativeMove(0, delta);
        else
          sim->relativeMove(0, iDelta);
        std::cerr << "s";
        sim->sense(0);
      }
    }
  }

  for (int j = 0; j < l; j++) {
    Isometry3 delta(j != l - 1 ? iq.toRotationMatrix() : Matrix3::Identity());
    delta.translation() = Vector3(delta_t * j, 0., 0.);
    Isometry3 iDelta = delta.inverse();
    for (int a = 0; a < 2; a++) {
      for (int i = 0; i < k; i++) {
        std::cerr << "m";
        if (a == 0)
          sim->relativeMove(0, delta);
        else
          sim->relativeMove(0, iDelta);
        std::cerr << "s";
        sim->sense(0);
      }
    }
  }

  std::ofstream os("test_gt.g2o");
  g->save(os);

  if (fixSensor) {
    ps->_offsetVertex->setFixed(true);
  } else {
    Vector6 noffcov;
    noffcov << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5;
    ps->_offsetVertex->setEstimate(ps->_offsetVertex->estimate() *
                                   sample_noise_from_se3(noffcov));
    ps->_offsetVertex->setFixed(false);
  }

  if (fixFirstPose) {
    auto gauge = g->vertex(4);
    if (gauge) gauge->setFixed(true);
  }  // else {
  //   // multiply all vertices of the robot by this standard quantity
  //   Quaternion q(AngleAxis(1, Vector3::UnitZ()).toRotationMatrix());
  //   Vector3 tr(1,0,0);
  //   Isometry3 delta;
  //   delta.matrix().block<3,3>(0,0)=q.toRotationMatrix();
  //   delta.translation()=tr;
  //   for (size_t i=0; i< g->vertices().size(); i++){
  //     VertexSE3 *v = dynamic_cast<VertexSE3 *>(g->vertex(i));
  //     if (v && v->id()>0){
  //   v->setEstimate (v->estimate()*delta);
  //     }
  //   }
  // }

  std::ofstream osp("test_preopt.g2o");
  g->save(osp);

  return 0;
}

}  // namespace g2o

int main(int argc, char** argv) { return g2o::simulator_3d_plane(argc, argv); }
