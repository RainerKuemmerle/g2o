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
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"

#include <iostream>

using namespace g2o;
using namespace std;
using namespace Eigen;

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

double uniform_rand(double lowerBndr, double upperBndr)
{
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

double gauss_rand(double sigma)
{
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

Eigen::Isometry3d sample_noise_from_se3(const Vector6& cov ){
  double nx=gauss_rand(cov(0));
  double ny=gauss_rand(cov(1));
  double nz=gauss_rand(cov(2));

  double nroll=gauss_rand(cov(3));
  double npitch=gauss_rand(cov(4));
  double nyaw=gauss_rand(cov(5));

  AngleAxisd aa(AngleAxisd(nyaw, Vector3d::UnitZ())*
    AngleAxisd(nroll, Vector3d::UnitX())*
    AngleAxisd(npitch, Vector3d::UnitY()));

  Eigen::Isometry3d retval=Isometry3d::Identity();
  retval.matrix().block<3,3>(0,0)=  aa.toRotationMatrix();
  retval.translation()=Vector3d(nx,ny,nz);
  return retval;
}

Vector3d sample_noise_from_plane(const Vector3d& cov ){
  return Vector3d(gauss_rand(cov(0)), gauss_rand(cov(1)), gauss_rand(cov(2)));
}

struct SimulatorItem {
  SimulatorItem(OptimizableGraph* graph_): _graph(graph_){}
  OptimizableGraph* graph() {return _graph;}
  virtual ~SimulatorItem(){}
  protected:
  OptimizableGraph* _graph;
};

struct WorldItem: public SimulatorItem {
  WorldItem(OptimizableGraph* graph_, OptimizableGraph::Vertex* vertex_ = 0) :
    SimulatorItem(graph_),_vertex(vertex_) {}
  OptimizableGraph::Vertex* vertex() {return _vertex;}
  void  setVertex(OptimizableGraph::Vertex* vertex_) {_vertex = vertex_;}
protected:
  OptimizableGraph::Vertex* _vertex;
};

typedef std::set<WorldItem*> WorldItemSet;

struct Robot;

struct Sensor {
  Sensor(Robot* robot_) : _robot(robot_){}
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

  Robot(OptimizableGraph* graph_): WorldItem(graph_) {
    _planarMotion=false;
    _position = Isometry3d::Identity();
  }


  void move(const Isometry3d& newPosition, int& id) {
    Isometry3d delta = _position.inverse()*newPosition;
    _position = newPosition;
    VertexSE3* v=new VertexSE3();
    v->setId(id);
    id++;
    graph()->addVertex(v);
    if (_planarMotion){
      // add a singleton constraint that locks the position of the robot on the plane
      EdgeSE3Prior* planeConstraint=new EdgeSE3Prior();
      Matrix6 pinfo = Matrix6::Zero();
      pinfo(2,2)=1e9;
      planeConstraint->setInformation(pinfo);
      planeConstraint->setMeasurement(Isometry3d::Identity());
      planeConstraint->vertices()[0]=v;
      planeConstraint->setParameterId(0,0);
      graph()->addEdge(planeConstraint);
    }
    if (vertex()){
      VertexSE3* oldV=dynamic_cast<VertexSE3*>(vertex());
      EdgeSE3* e=new EdgeSE3();
      Isometry3d noise=sample_noise_from_se3(_nmovecov);
      e->setMeasurement(delta*noise);
      Matrix6 m=Matrix6::Identity();
      for (int i=0; i<6; i++){
	m(i,i)=1./(_nmovecov(i));
      }
      e->setInformation(m);
      e->vertices()[0]=vertex();
      e->vertices()[1]=v;
      graph()->addEdge(e);
      v->setEstimate(oldV->estimate()*e->measurement());
    } else {
      v->setEstimate(_position);
    }
    setVertex(v);
  }

  void relativeMove(const Isometry3d& delta, int& id){
    Isometry3d newPosition = _position*delta;
    move(newPosition, id);
  }

  void sense(WorldItem* wi=0){
    for (size_t i=0; i<_sensors.size(); i++){
      Sensor* s=_sensors[i];
      s->sense(wi, _position);
    }
  }

  Isometry3d _position;
  SensorVector _sensors;
  Vector6 _nmovecov;
  bool _planarMotion;
};

typedef std::vector<Robot*> RobotVector;

struct Simulator: public SimulatorItem {
  Simulator(OptimizableGraph* graph_): SimulatorItem(graph_), _lastVertexId(0){}
  void sense(int robotIndex){
    Robot* r=_robots[robotIndex];
    for (WorldItemSet::iterator it=_world.begin(); it!=_world.end(); it++){
      WorldItem* item=*it;
      r->sense(item);
    }
  }

  void move(int robotIndex, const Isometry3d& newRobotPose){
    Robot* r=_robots[robotIndex];
    r->move(newRobotPose, _lastVertexId);
  }

  void relativeMove(int robotIndex, const Isometry3d& delta){
    Robot* r=_robots[robotIndex];
    r->relativeMove(delta, _lastVertexId);
  }

  int _lastVertexId;
  WorldItemSet _world;
  RobotVector _robots;
};

struct PlaneItem: public WorldItem{
  PlaneItem(OptimizableGraph* graph_, int id) : WorldItem(graph_){
    VertexPlane* p=new VertexPlane();
    p->setId(id);
    graph()->addVertex(p);
    setVertex(p);
  }
};

struct PlaneSensor: public Sensor{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PlaneSensor(Robot* r, int offsetId, const Isometry3d& offset_): Sensor(r){
    _offsetVertex = new VertexSE3();
    _offsetVertex->setId(offsetId);
    _offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(_offsetVertex);
  };

  virtual bool isVisible(const WorldItem* wi) const {
    if (! wi)
      return false;
    const PlaneItem* pi=dynamic_cast<const PlaneItem*>(wi);
    if (! pi)
      return false;
    return true;
  }

  virtual bool sense(WorldItem* wi, const Isometry3d& position){
    if (! wi)
      return false;
    PlaneItem* pi=dynamic_cast<PlaneItem*>(wi);
    if (! pi)
      return false;
    OptimizableGraph::Vertex* rv = robot()->vertex();
    if (! rv) {
      return false;
    }
    VertexSE3* robotVertex = dynamic_cast<VertexSE3*>(rv);
    if (! robotVertex){
      return false;
    }
    const Isometry3d& robotPose=position;
    Isometry3d sensorPose=robotPose*_offsetVertex->estimate();
    VertexPlane* planeVertex=dynamic_cast<VertexPlane*>(pi->vertex());
    Plane3D worldPlane=planeVertex->estimate();

    Plane3D measuredPlane=sensorPose.inverse()*worldPlane;

    EdgeSE3PlaneSensorCalib* e=new EdgeSE3PlaneSensorCalib();
    e->vertices()[0]=robotVertex;
    e->vertices()[1]=planeVertex;
    e->vertices()[2]=_offsetVertex;
    Vector3d noise = sample_noise_from_plane(_nplane);
    measuredPlane.oplus(noise);
    e->setMeasurement(measuredPlane);
    Matrix3d m=Matrix3d::Zero();
    m(0,0)=1./(_nplane(0));
    m(1,1)=1./(_nplane(1));
    m(2,2)=1./(_nplane(2));
    e->setInformation(m);
    robot()->graph()->addEdge(e);
    return true;
  }

  VertexSE3* _offsetVertex;
  Vector3d _nplane;
};

int main (int argc  , char ** argv){
  int maxIterations;
  bool verbose;
  bool robustKernel;
  double lambdaInit;
  CommandArgs arg;
  bool fixSensor;
  bool fixPlanes;
  bool fixFirstPose;
  bool fixTrajectory;
  bool planarMotion;
  bool listSolvers;
  string strSolver;
  cerr << "graph" << endl;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("solver", strSolver, "lm_var", "select one specific solver");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("fixSensor", fixSensor, false, "fix the sensor position on the robot");
  arg.param("fixTrajectory", fixTrajectory, false, "fix the trajectory");
  arg.param("fixFirstPose", fixFirstPose, false, "fix the first robot pose");
  arg.param("fixPlanes", fixPlanes, false, "fix the planes (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.param("listSolvers", listSolvers, false, "list the solvers");
  arg.parseArgs(argc, argv);



  SparseOptimizer* g=new SparseOptimizer();
  ParameterSE3Offset* odomOffset=new ParameterSE3Offset();
  odomOffset->setId(0);
  g->addParameter(odomOffset);

  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct(strSolver, solverProperty);
  g->setAlgorithm(solver);
  if (listSolvers){
    solverFactory->listSolvers(cerr);
    return 0;
  }

  if (! g->solver()){
    cerr << "Error allocating solver. Allocating \"" << strSolver << "\" failed!" << endl;
    cerr << "available solvers: " << endl;
    solverFactory->listSolvers(cerr);
    cerr << "--------------" << endl;
    return 0;
  }

  cerr << "sim" << endl;
  Simulator* sim = new Simulator(g);

  cerr << "robot" << endl;
  Robot* r=new Robot(g);


  cerr << "planeSensor" << endl;
  Matrix3d R=Matrix3d::Identity();
  R <<
    0,  0,   1,
    -1,  0,  0,
    0, -1,   0;

  Isometry3d sensorPose=Isometry3d::Identity();
  sensorPose.matrix().block<3,3>(0,0) = R;
  sensorPose.translation()= Vector3d(.3 , 0.5 , 1.2);
  PlaneSensor* ps = new PlaneSensor(r, 0, sensorPose);
  ps->_nplane << 0.03, 0.03, 0.005;
  r->_sensors.push_back(ps);
  sim->_robots.push_back(r);

  cerr  << "p1" << endl;
  Plane3D plane;
  PlaneItem* pi =new PlaneItem(g,1);
  plane.fromVector(Eigen::Vector4d(0.,0.,1.,5.));
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);

  plane.fromVector(Eigen::Vector4d(1.,0.,0.,5.));
  pi =new PlaneItem(g,2);
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);

  cerr  << "p2" << endl;
  pi =new PlaneItem(g,3);
  plane.fromVector(Eigen::Vector4d(0.,1.,0.,5.));
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);


  Quaterniond q, iq;
  if (planarMotion) {
    r->_planarMotion = true;
    r->_nmovecov << 0.01, 0.0025, 1e-9, 0.001, 0.001, 0.025;
    q = Quaterniond(AngleAxisd(0.2, Vector3d::UnitZ()).toRotationMatrix());
    iq = Quaterniond(AngleAxisd(-0.2, Vector3d::UnitZ()).toRotationMatrix());
  } else {
    r->_planarMotion = false;
    //r->_nmovecov << 0.1, 0.005, 1e-9, 0.05, 0.001, 0.001;
    r->_nmovecov << 0.1, 0.005, 1e-9, 0.001, 0.001, 0.05;
    q = Quaterniond((AngleAxisd(M_PI/10, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitY())).toRotationMatrix());
    iq = Quaterniond((AngleAxisd(-M_PI/10, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitY())).toRotationMatrix());
  }

  Isometry3d delta=Isometry3d::Identity();
  sim->_lastVertexId=4;

  Isometry3d startPose=Isometry3d::Identity();
  startPose.matrix().block<3,3>(0,0) = AngleAxisd(-0.75*M_PI, Vector3d::UnitZ()).toRotationMatrix();
  sim->move(0,startPose);

  int k =20;
  int l = 2;
  double delta_t = 0.2;
  for (int j=0; j<l; j++) {
    Vector3d tr(1.,0.,0.);
    delta.matrix().block<3,3>(0,0) = q.toRotationMatrix();
    if (j==(l-1)){
      delta.matrix().block<3,3>(0,0) = Matrix3d::Identity();
    }
    delta.translation()=tr*(delta_t*j);
    Isometry3d iDelta = delta.inverse();
    for (int a=0; a<2; a++){
      for (int i=0; i<k; i++){
        cerr << "m";
        if (a==0)
          sim->relativeMove(0,delta);
        else
          sim->relativeMove(0,iDelta);
        cerr << "s";
        sim->sense(0);
      }
    }
  }

  for (int j=0; j<l; j++) {
    Vector3d tr(1.,0.,0.);
    delta.matrix().block<3,3>(0,0) = iq.toRotationMatrix();
    if (j==l-1){
      delta.matrix().block<3,3>(0,0) = Matrix3d::Identity();
    }
    delta.translation()=tr*(delta_t*j);
    Isometry3d iDelta = delta.inverse();
    for (int a=0; a<2; a++){
      for (int i=0; i<k; i++){
        cerr << "m";
        if (a==0)
          sim->relativeMove(0,delta);
        else
          sim->relativeMove(0,iDelta);
        cerr << "s";
        sim->sense(0);
      }
    }
  }

  ofstream os("test_gt.g2o");
  g->save(os);

  if (fixSensor) {
    ps->_offsetVertex->setFixed(true);
  } else {
    Vector6 noffcov;
    noffcov << 0.1,0.1,0.1,0.5, 0.5, 0.5;
    ps->_offsetVertex->setEstimate(ps->_offsetVertex->estimate() * sample_noise_from_se3(noffcov));
    ps->_offsetVertex->setFixed(false);
  }

  if (fixFirstPose){
    OptimizableGraph::Vertex* gauge = g->vertex(4);
    if (gauge)
      gauge->setFixed(true);
  } // else {
  //   // multiply all vertices of the robot by this standard quantity
  //   Quaterniond q(AngleAxisd(1, Vector3d::UnitZ()).toRotationMatrix());
  //   Vector3d tr(1,0,0);
  //   Isometry3d delta;
  //   delta.matrix().block<3,3>(0,0)=q.toRotationMatrix();
  //   delta.translation()=tr;
  //   for (size_t i=0; i< g->vertices().size(); i++){
  //     VertexSE3 *v = dynamic_cast<VertexSE3 *>(g->vertex(i));
  //     if (v && v->id()>0){
  //   v->setEstimate (v->estimate()*delta);
  //     }
  //   }
  // }

  ofstream osp("test_preopt.g2o");
  g->save(osp);
  //g->setMethod(SparseOptimizer::LevenbergMarquardt);
  g->initializeOptimization();
  g->setVerbose(verbose);
  g->optimize(maxIterations);
  if (! fixSensor ){
    SparseBlockMatrix<MatrixXd> spinv;
    std::pair<int, int> indexParams;
    indexParams.first = ps->_offsetVertex->hessianIndex();
    indexParams.second = ps->_offsetVertex->hessianIndex();
    std::vector<std::pair <int, int> > blockIndices;
    blockIndices.push_back(indexParams);
    if (!g->computeMarginals(spinv,  blockIndices)){
      cerr << "error in computing the covariance" << endl;
    } else {

      MatrixXd m = *spinv.block(ps->_offsetVertex->hessianIndex(), ps->_offsetVertex->hessianIndex());

      cerr << "Param covariance" << endl;
      cerr << m << endl;
      cerr << "OffsetVertex: " << endl;
      ps->_offsetVertex->write(cerr);
      cerr <<  endl;
      cerr << "rotationDeterminant: " << m.block<3,3>(0,0).determinant() << endl;
      cerr << "translationDeterminant: " << m.block<3,3>(3,3).determinant()  << endl;
      cerr << endl;
    }
  }
  ofstream os1("test_postOpt.g2o");
  g->save(os1);

}
