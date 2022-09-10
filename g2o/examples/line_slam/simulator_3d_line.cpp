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
  double nx = Sampler::gaussRand(0., cov(0));
  double ny = Sampler::gaussRand(0., cov(1));
  double nz = Sampler::gaussRand(0., cov(2));

  double nroll = Sampler::gaussRand(0., cov(3));
  double npitch = Sampler::gaussRand(0., cov(4));
  double nyaw = Sampler::gaussRand(0., cov(5));

  AngleAxis aa(AngleAxis(nyaw, Vector3::UnitZ()) *
               AngleAxis(nroll, Vector3::UnitX()) *
               AngleAxis(npitch, Vector3::UnitY()));

  auto retval = Isometry3::Identity();
  retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
  retval.translation() = Vector3(nx, ny, nz);
  return retval;
}

static Vector4 sample_noise_from_line(const Vector4& cov) {
  return Vector4(Sampler::gaussRand(0., cov(0)), Sampler::gaussRand(0., cov(1)),
                 Sampler::gaussRand(0., cov(2)),
                 Sampler::gaussRand(0., cov(3)));
}

struct SimulatorItem {
  explicit SimulatorItem(OptimizableGraph* graph_) : graph_(graph_) {}
  OptimizableGraph* graph() { return graph_; }
  virtual ~SimulatorItem() = default;

 protected:
  OptimizableGraph* graph_;
};

struct WorldItem : public SimulatorItem {
  explicit WorldItem(OptimizableGraph* graph,
                     std::shared_ptr<OptimizableGraph::Vertex> vertex = nullptr)
      : SimulatorItem(graph), vertex_(std::move(vertex)) {}
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

  explicit Robot(OptimizableGraph* graph) : WorldItem(graph) {
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
      Matrix6 m = Vector6(1.0 / nmovecov.array()).asDiagonal();
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

struct LineItem : public WorldItem {
  LineItem(OptimizableGraph* graph_, int id) : WorldItem(graph_) {
    auto l = std::make_shared<VertexLine3D>();
    l->setId(id);
    graph()->addVertex(l);
    setVertex(l);
  }
};

struct LineSensor : public Sensor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  LineSensor(Robot* r, int offsetId, const Isometry3& offset_) : Sensor(r) {
    offsetVertex = std::make_shared<VertexSE3>();
    offsetVertex->setId(offsetId);
    offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(offsetVertex);
  };

  bool isVisible(const WorldItem* wi) const override {
    if (!wi) {
      return false;
    }
    const auto* li = dynamic_cast<const LineItem*>(wi);
    return li != nullptr;
  }

  bool sense(WorldItem* wi, const Isometry3& position) override {
    if (!wi) {
      return false;
    }
    auto* li = dynamic_cast<LineItem*>(wi);
    if (!li) {
      return false;
    }
    auto rv = robot()->vertex();
    if (!rv) {
      return false;
    }
    auto robotVertex = std::dynamic_pointer_cast<VertexSE3>(rv);
    if (!robotVertex) {
      return false;
    }
    const Isometry3& robotPose = position;
    Isometry3 sensorPose = robotPose * offsetVertex->estimate();
    auto lineVertex = std::dynamic_pointer_cast<VertexLine3D>(li->vertex());
    Line3D worldLine = lineVertex->estimate();

    Line3D measuredLine = sensorPose.inverse() * worldLine;

    auto e = std::make_shared<EdgeSE3Line3D>();
    e->vertices()[0] = robotVertex;
    e->vertices()[1] = lineVertex;
    Vector4 noise = sample_noise_from_line(nline);
    measuredLine.oplus(noise);
    e->setMeasurement(measuredLine);
    Matrix4 m = Vector4(1.0 / nline.array()).asDiagonal();
    e->setInformation(m);
    e->setParameterId(0, 0);
    robot()->graph()->addEdge(e);
    return true;
  }

  std::shared_ptr<VertexSE3> offsetVertex;
  Vector4 nline;
};

static int simulator_3d_line(int argc, char** argv) {
  bool fixLines;
  bool planarMotion;
  CommandArgs arg;
  arg.param("fixLines", fixLines, false,
            "fix the lines (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.parseArgs(argc, argv);

  auto* g = new SparseOptimizer();
  auto odomOffset = std::make_shared<ParameterSE3Offset>();
  odomOffset->setId(0);
  g->addParameter(odomOffset);

  std::cout << "Creating simulator" << std::endl;
  auto* sim = new Simulator(g);

  std::cout << "Creating robot" << std::endl;
  auto* r = new Robot(g);

  std::cout << "Creating line sensor" << std::endl;
  Isometry3 sensorPose = Isometry3::Identity();
  auto* ls = new LineSensor(r, 0, sensorPose);
  ls->nline = Vector4(0.001, 0.001, 0.001, 0.0001);
  // ls->nline << 1e-9, 1e-9, 1e-9, 1e-9;
  r->sensors.push_back(ls);
  sim->robots.push_back(r);

  Line3D line;
  std::cout << "Creating landmark line 1" << std::endl;
  auto* li = new LineItem(g, 1);
  Vector6 liv;
  liv << 0.0, 0.0, 5.0, 0.0, 1.0, 0.0;
  line = Line3D::fromCartesian(liv);
  static_cast<VertexLine3D*>(li->vertex().get())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->world.insert(li);

  std::cout << "Creating landmark line 2" << std::endl;
  liv << 5.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  line = Line3D::fromCartesian(liv);
  li = new LineItem(g, 2);
  static_cast<VertexLine3D*>(li->vertex().get())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->world.insert(li);

  std::cout << "Creating landmark line 3" << std::endl;
  liv << 0.0, 5.0, 0.0, 1.0, 0.0, 0.0;
  line = Line3D::fromCartesian(liv);
  li = new LineItem(g, 3);
  static_cast<VertexLine3D*>(li->vertex().get())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->world.insert(li);

  Quaternion q;
  Quaternion iq;
  if (planarMotion) {
    r->planarMotion = true;
    r->nmovecov << 0.01, 0.0025, 1e-9, 0.001, 0.001, 0.025;
    q = Quaternion(AngleAxis(0.2, Vector3::UnitZ()));
    iq = Quaternion(AngleAxis(-0.2, Vector3::UnitZ()));
  } else {
    r->planarMotion = false;
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
  for (int j = 0; j < l; ++j) {
    Isometry3 delta(l - 1 ? q.toRotationMatrix() : Matrix3::Identity());
    delta.translation() = Vector3(delta_t * j, 0.0, 0.0);
    Isometry3 iDelta = delta.inverse();
    for (int a = 0; a < 2; ++a) {
      for (int i = 0; i < k; ++i) {
        std::cout << "m";
        sim->relativeMove(0, a == 0 ? delta : iDelta);
        std::cout << "s";
        sim->sense(0);
      }
    }
  }
  for (int j = 0; j < l; ++j) {
    Isometry3 delta(j != l - 1 ? iq.toRotationMatrix() : Matrix3::Identity());
    delta.translation() = Vector3(delta_t * j, 0.0, 0.0);
    Isometry3 iDelta = delta.inverse();
    for (int a = 0; a < 2; ++a) {
      for (int i = 0; i < k; ++i) {
        std::cout << "m";
        sim->relativeMove(0, a == 0 ? delta : iDelta);
        std::cout << "s";
        sim->sense(0);
      }
    }
  }
  std::cout << std::endl;

  ls->offsetVertex->setFixed(true);
  auto gauge = g->vertex(4);
  if (gauge) {
    gauge->setFixed(true);
  }

  std::ofstream osp("line3d.g2o");
  g->save(osp);
  std::cout << "Saved graph on file line3d.g2o, use g2o_viewer to work with it."
            << std::endl;

  return 0;
}
}  // namespace g2o

int main(int argc, char** argv) { return g2o::simulator_3d_line(argc, argv); }
