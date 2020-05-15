#include <iostream>
#include <fstream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"

using namespace g2o;
using namespace std;
using namespace Eigen;

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

Eigen::Isometry3d sample_noise_from_se3(const Vector6& cov ) {
  double nx = Sampler::gaussRand(0., cov(0));
  double ny = Sampler::gaussRand(0., cov(1));
  double nz = Sampler::gaussRand(0., cov(2));

  double nroll = Sampler::gaussRand(0., cov(3));
  double npitch = Sampler::gaussRand(0., cov(4));
  double nyaw = Sampler::gaussRand(0., cov(5));

  AngleAxisd aa(AngleAxisd(nyaw, Vector3d::UnitZ())*
		AngleAxisd(nroll, Vector3d::UnitX())*
		AngleAxisd(npitch, Vector3d::UnitY()));

  Eigen::Isometry3d retval = Isometry3d::Identity();
  retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
  retval.translation() = Vector3d(nx, ny, nz);
  return retval;
}

Vector4d sample_noise_from_line(const Vector4d& cov) {
  return Vector4d(Sampler::gaussRand(0., cov(0)), Sampler::gaussRand(0., cov(1)),
                  Sampler::gaussRand(0., cov(2)), Sampler::gaussRand(0., cov(3)));
}

struct SimulatorItem {
  SimulatorItem(OptimizableGraph* graph_): _graph(graph_) {}
  OptimizableGraph* graph() { return _graph;}
  virtual ~SimulatorItem(){}
protected:
  OptimizableGraph* _graph;
};

struct WorldItem : public SimulatorItem {
  WorldItem(OptimizableGraph* graph_, OptimizableGraph::Vertex* vertex_ = 0) :
    SimulatorItem(graph_),_vertex(vertex_) {}
  OptimizableGraph::Vertex* vertex() { return _vertex; }
  void  setVertex(OptimizableGraph::Vertex* vertex_) { _vertex = vertex_; }
protected:
  OptimizableGraph::Vertex* _vertex;
};

typedef std::set<WorldItem*> WorldItemSet;

struct Robot;

struct Sensor {
  Sensor(Robot* robot_) : _robot(robot_) {}
  Robot* robot() {return _robot;}
  virtual bool isVisible(const WorldItem* ) const {return false;}
  virtual bool sense(WorldItem* , const Isometry3d& ) {return false;}
  virtual ~Sensor(){};
protected:
  Robot* _robot;
};

typedef std::vector<Sensor*> SensorVector;

struct Robot: public WorldItem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Robot(OptimizableGraph* graph_) : WorldItem(graph_) {
    _planarMotion = false;
    _position = Isometry3d::Identity();
  }

  void move(const Isometry3d& newPosition, int& id) {
    Isometry3d delta = _position.inverse() * newPosition;
    _position = newPosition;
    VertexSE3* v = new VertexSE3();
    v->setId(id);
    id++;
    graph()->addVertex(v);
    if(_planarMotion) {
      // add a singleton constraint that locks the position of the robot on the plane
      EdgeSE3Prior* planeConstraint=new EdgeSE3Prior();
      Matrix6 pinfo = Matrix6::Zero();
      pinfo(2, 2) = 1e9;
      planeConstraint->setInformation(pinfo);
      planeConstraint->setMeasurement(Isometry3d::Identity());
      planeConstraint->vertices()[0] = v;
      planeConstraint->setParameterId(0, 0);
      graph()->addEdge(planeConstraint);
    }
    if(vertex()) {
      VertexSE3* oldV = dynamic_cast<VertexSE3*>(vertex());
      EdgeSE3* e = new EdgeSE3();
      Isometry3d noise = sample_noise_from_se3(_nmovecov);
      e->setMeasurement(delta * noise);
      Matrix6 m = Matrix6::Identity();
      for(int i = 0; i < 6; ++i) {
	      m(i, i) = 1.0 / (_nmovecov(i));
      }
      e->setInformation(m);
      e->vertices()[0] = vertex();
      e->vertices()[1] = v;
      graph()->addEdge(e);
      v->setEstimate(oldV->estimate() * e->measurement());
    }
    else {
      v->setEstimate(_position);
    }
    setVertex(v);
  }

  void relativeMove(const Isometry3d& delta, int& id) {
    Isometry3d newPosition = _position * delta;
    move(newPosition, id);
  }

  void sense(WorldItem* wi = 0) {
    for(size_t i = 0; i < _sensors.size(); ++i) {
      Sensor* s = _sensors[i];
      s->sense(wi, _position);
    }
  }

  Isometry3d _position;
  SensorVector _sensors;
  Vector6 _nmovecov;
  bool _planarMotion;
};

typedef std::vector<Robot*> RobotVector;

struct Simulator : public SimulatorItem {
  Simulator(OptimizableGraph* graph_) : SimulatorItem(graph_), _lastVertexId(0) {}
  void sense(int robotIndex) {
    Robot* r = _robots[robotIndex];
    for(WorldItemSet::iterator it = _world.begin(); it != _world.end(); ++it) {
      WorldItem* item = *it;
      r->sense(item);
    }
  }

  void move(int robotIndex, const Isometry3d& newRobotPose) {
    Robot* r = _robots[robotIndex];
    r->move(newRobotPose, _lastVertexId);
  }

  void relativeMove(int robotIndex, const Isometry3d& delta) {
    Robot* r = _robots[robotIndex];
    r->relativeMove(delta, _lastVertexId);
  }

  int _lastVertexId;
  WorldItemSet _world;
  RobotVector _robots;
};

struct LineItem : public WorldItem {
  LineItem(OptimizableGraph* graph_, int id) : WorldItem(graph_) {
    VertexLine3D* l = new VertexLine3D();
    l->setId(id);
    graph()->addVertex(l);
    setVertex(l);
  }
};

struct LineSensor : public Sensor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  LineSensor(Robot* r, int offsetId, const Isometry3d& offset_) : Sensor(r) {
    _offsetVertex = new VertexSE3();
    _offsetVertex->setId(offsetId);
    _offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(_offsetVertex);
  };

  virtual bool isVisible(const WorldItem* wi) const {
    if(!wi) {
      return false;
    }
    const LineItem* li = dynamic_cast<const LineItem*>(wi);
    if(!li) {
      return false;
    }
    return true;
  }

  virtual bool sense(WorldItem* wi, const Isometry3d& position) {
    if(!wi) {
      return false;
    }
    LineItem* li = dynamic_cast<LineItem*>(wi);
    if(!li) {
      return false;
    }
    OptimizableGraph::Vertex* rv = robot()->vertex();
    if(!rv) {
      return false;
    }
    VertexSE3* robotVertex = dynamic_cast<VertexSE3*>(rv);
    if(!robotVertex) {
      return false;
    }
    const Isometry3d& robotPose = position;
    Isometry3d sensorPose = robotPose * _offsetVertex->estimate();
    VertexLine3D* lineVertex = dynamic_cast<VertexLine3D*>(li->vertex());
    Line3D worldLine = lineVertex->estimate();

    Line3D measuredLine = sensorPose.inverse() * worldLine;

    EdgeSE3Line3D* e = new EdgeSE3Line3D();
    e->vertices()[0] = robotVertex;
    e->vertices()[1] = lineVertex;
    Vector4d noise = sample_noise_from_line(_nline);
    measuredLine.oplus(noise);
    e->setMeasurement(measuredLine);
    Matrix4d m = Matrix4d::Zero();
    m(0, 0) = 1.0 / (_nline(0));
    m(1, 1) = 1.0 / (_nline(1));
    m(2, 2) = 1.0 / (_nline(2));
    m(3, 3) = 1.0 / (_nline(3));
    e->setInformation(m);
    e->setParameterId(0, 0);
    robot()->graph()->addEdge(e);
    return true;
  }

  VertexSE3* _offsetVertex;
  Vector4d _nline;
};

int main (int argc, char** argv) {
  bool verbose, robustKernel, fixLines, planarMotion, listSolvers;
  int maxIterations;
  double lambdaInit;
  string strSolver;
  CommandArgs arg;
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("solver", strSolver, "lm_var", "select one specific solver");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("fixLines", fixLines, false, "fix the lines (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.param("listSolvers", listSolvers, false, "list the solvers");
  arg.parseArgs(argc, argv);

  SparseOptimizer* g = new SparseOptimizer();
  ParameterSE3Offset* odomOffset = new ParameterSE3Offset();
  odomOffset->setId(0);
  g->addParameter(odomOffset);

  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct(strSolver, solverProperty);
  g->setAlgorithm(solver);
  if(listSolvers) {
    solverFactory->listSolvers(std::cout);
    return 0;
  }

  if(!g->solver()) {
    std::cout << "Error allocating solver. Allocating \"" << strSolver << "\" failed!" << std::endl;
    std::cout << "Available solvers: " << std::endl;
    solverFactory->listSolvers(std::cout);
    std::cout << "--------------" << std::endl;
    return 0;
  }

  std::cout << "Creating simulator" << std::endl;
  Simulator* sim = new Simulator(g);

  std::cout << "Creating robot" << std::endl;
  Robot* r = new Robot(g);

  std::cout << "Creating line sensor" << std::endl;
  Isometry3d sensorPose = Isometry3d::Identity();
  LineSensor* ls = new LineSensor(r, 0, sensorPose);
  ls->_nline << 0.001, 0.001, 0.001, 0.0001;
  // ls->_nline << 1e-9, 1e-9, 1e-9, 1e-9;
  r->_sensors.push_back(ls);
  sim->_robots.push_back(r);

  Line3D line;
  std::cout << "Creating landmark line 1" << std::endl;
  LineItem* li = new LineItem(g, 1);
  Vector6 liv;
  liv << 0.0, 0.0, 5.0, 0.0, 1.0, 0.0;
  line = Line3D::fromCartesian(liv);
  static_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(li);

  std::cout << "Creating landmark line 2" << std::endl;
  liv << 5.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  line = Line3D::fromCartesian(liv);
  li = new LineItem(g, 2);
  static_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(li);

  std::cout << "Creating landmark line 3" << std::endl;
  liv << 0.0, 5.0, 0.0, 1.0, 0.0, 0.0;
  line = Line3D::fromCartesian(liv);
  li = new LineItem(g, 3);
  static_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(li);

  Quaterniond q, iq;
  if(planarMotion) {
    r->_planarMotion = true;
    r->_nmovecov << 0.01, 0.0025, 1e-9, 0.001, 0.001, 0.025;
    q = Quaterniond(AngleAxisd(0.2, Vector3d::UnitZ()).toRotationMatrix());
    iq = Quaterniond(AngleAxisd(-0.2, Vector3d::UnitZ()).toRotationMatrix());
  }
  else {
    r->_planarMotion = false;
    r->_nmovecov << 0.1, 0.005, 1e-9, 0.001, 0.001, 0.05;
    q = Quaterniond((AngleAxisd(M_PI/10, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitY())).toRotationMatrix());
    iq = Quaterniond((AngleAxisd(-M_PI/10, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitY())).toRotationMatrix());
  }

  Isometry3d delta = Isometry3d::Identity();
  sim->_lastVertexId = 4;

  Isometry3d startPose = Isometry3d::Identity();
  startPose.matrix().block<3, 3>(0, 0) = AngleAxisd(-0.75 * M_PI, Vector3d::UnitZ()).toRotationMatrix();
  sim->move(0, startPose);

  int k = 20;
  int l = 2;
  double delta_t = 0.2;
  for(int j = 0; j < l; ++j) {
    Vector3d tr(1.0, 0.0, 0.0);
    delta.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
    if(j == l-1) {
      delta.matrix().block<3, 3>(0, 0) = Matrix3d::Identity();
    }
    delta.translation() = tr * (delta_t * j);
    Isometry3d iDelta = delta.inverse();
    for(int a = 0; a < 2; ++a) {
      for(int i = 0; i < k; ++i) {
        std::cout << "m";
        if(a == 0) {
          sim->relativeMove(0, delta);
	}
	else {
          sim->relativeMove(0, iDelta);
	}
	std::cout << "s";
        sim->sense(0);
      }
    }
  }
  for(int j = 0; j < l; ++j) {
    Vector3d tr(1.0, 0.0, 0.0);
    delta.matrix().block<3, 3>(0, 0) = iq.toRotationMatrix();
    if(j == l-1) {
      delta.matrix().block<3, 3>(0, 0) = Matrix3d::Identity();
    }
    delta.translation() = tr * (delta_t * j);
    Isometry3d iDelta = delta.inverse();
    for(int a = 0; a < 2; ++a) {
      for(int i = 0; i < k; ++i) {
        std::cout << "m";
        if(a == 0) {
          sim->relativeMove(0, delta);
  	}
        else {
          sim->relativeMove(0, iDelta);
  	}
        std::cout << "s";
        sim->sense(0);
      }
    }
  }
  std::cout << std::endl;

  ls->_offsetVertex->setFixed(true);
  OptimizableGraph::Vertex* gauge = g->vertex(4);
  if(gauge) {
    gauge->setFixed(true);
  }

  ofstream osp("line3d.g2o");
  g->save(osp);
  std::cout << "Saved graph on file line3d.g2o, use g2o_viewer to work with it." << std::endl;

  return 0;
}
