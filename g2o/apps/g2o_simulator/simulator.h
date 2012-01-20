#ifndef G2O_SIMULATOR_
#define G2O_SIMULATOR_

#include <string>
#include <set>
#include "g2o/config.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/stuff/sampler.h"
#include "g2o_simulator_api.h"

namespace g2o{

class World;
class BaseSensor;

class G2O_SIMULATOR_API BaseWorldObject{
 public:
  BaseWorldObject(World* world_=0) {_world = world_; _vertex=0;}
  virtual ~BaseWorldObject();
  void setWorld(World* world_) {_world = world_;}
  World* world() {return _world;}
  OptimizableGraph* graph();
  OptimizableGraph::Vertex* vertex() {return _vertex;}
  virtual void setVertex(OptimizableGraph::Vertex* vertex_);
 protected:
  OptimizableGraph* _graph;
  OptimizableGraph::Vertex* _vertex;
  World* _world;
};

template <class VertexType_>
class WorldObject: public BaseWorldObject, VertexType_{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  typedef VertexType_ VertexType;
  typedef typename VertexType_::EstimateType EstimateType;
  WorldObject(World* world_=0): BaseWorldObject(world_){
    _vertex = new VertexType();
  } 
  virtual void setVertex(OptimizableGraph::Vertex* vertex_){
    if(! dynamic_cast<VertexType*>(vertex_))
      return;
    _vertex = vertex_;
  }

  VertexType* vertex() {
    if (! _vertex) return 0; 
    return dynamic_cast<VertexType*>(_vertex);
  }
};

class G2O_SIMULATOR_API BaseRobot {
 public:
  BaseRobot(World* world_, const std::string& name_){_world = world_; _name = name_; }
  void setWorld(World* world_) {_world = world_;}
  World* world() {return _world;}
  const std::string& name() const {return _name;}
  OptimizableGraph* graph();
  bool addSensor(BaseSensor* sensor);
  const std::set<BaseSensor*> sensors() {return _sensors;}
  virtual void sense();
 protected:
  World* _world;
  std::set<BaseSensor*> _sensors;
  std::string _name;
};

template <class RobotPoseObject>
class Robot: public BaseRobot{
 public:
  typedef RobotPoseObject PoseObject;
  typedef std::list<PoseObject*> TrajectoryType;
  typedef typename PoseObject::VertexType VertexType;
  typedef typename PoseObject::EstimateType PoseType;
  
 Robot(World* world_, const std::string& name_): BaseRobot(world_, name_){}
  virtual void relativeMove(const PoseType& movement_) {
    _pose=_pose*movement_;
    move(_pose);
  }
  
  virtual void move(const PoseType& pose_) {
    _pose=pose_;
    if (world()) {
      PoseObject* po=new PoseObject();
      po->vertex()->setEstimate(_pose);
      world()->addWorldObject(po);
      _trajectory.push_back(po);
    }
  }
  
  TrajectoryType& trajectory() {return _trajectory;}
  const PoseType& pose() const {return _pose;}
 protected:
  TrajectoryType _trajectory;
  PoseType _pose;
};
 
class G2O_SIMULATOR_API BaseSensor{
 public:
  BaseSensor(const std::string& name_){ _name = name_;}
  inline BaseRobot* robot() {return _robot;}
  inline void setRobot(BaseRobot* robot_) {_robot = robot_;}
  World* world();
  OptimizableGraph* graph();
  std::vector<Parameter*> parameters() {return _parameters;}
  virtual void sense() = 0;
  virtual void addParameters() {}
 protected:
  std::string _name;
  std::vector<Parameter*> _parameters;
  BaseRobot* _robot;
};

template <class RobotType_, class EdgeType_>
class UnarySensor: public BaseSensor {
 public:
  typedef RobotType_ RobotType;
  typedef typename RobotType::PoseObject PoseObject;
  typedef typename RobotType::TrajectoryType TrajectoryType;
  typedef typename RobotType::PoseObject::VertexType PoseVertexType;
  typedef EdgeType_ EdgeType;
  typedef typename EdgeType::InformationType InformationType;

  UnarySensor(const std::string& name): BaseSensor(name) {
   _information.setIdentity();
  }

  void setInformation(const InformationType& information_ ) {
    _information = information_ ;
    _sampler.setDistribution(_information.inverse());
  }

  const InformationType& information() {return _information; }

  virtual void sense() {
    _robotPoseObject = 0;
    // set the robot pose
    if (! robot())
      return;
    
    RobotType* r =dynamic_cast<RobotType*>(robot());
    if (!r)
      return;
    
    if(! r->trajectory().empty())
      _robotPoseObject = *(r->trajectory().rbegin());

    if (! world() || ! graph())
      return;
    
    EdgeType* e=mkEdge();
    if (e) {
      e->setMeasurementFromState();
      addNoise(e);
      graph()->addEdge(e);
    }
  }


 protected:
  PoseObject* _robotPoseObject;
  InformationType _information;

  EdgeType* mkEdge(){
    PoseVertexType* robotVertex = (PoseVertexType*)_robotPoseObject->vertex();
    EdgeType* e = new EdgeType();
    e->vertices()[0]=robotVertex;
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> _sampler;
  virtual void addNoise(EdgeType*){};
};

template <class RobotType_, class EdgeType_, class WorldObjectType_>
class BinarySensor: public BaseSensor {
 public:
  typedef RobotType_ RobotType;
  typedef typename RobotType::PoseObject PoseObject;
  typedef typename RobotType::TrajectoryType TrajectoryType;
  typedef typename RobotType::PoseObject::VertexType PoseVertexType;
  typedef EdgeType_ EdgeType;
  typedef WorldObjectType_ WorldObjectType;
  typedef typename WorldObjectType::VertexType VertexType;
  typedef typename EdgeType::InformationType InformationType;

  BinarySensor(const std::string& name): BaseSensor(name) {
   _information.setIdentity();
  }

  void setInformation(const InformationType& information_ ) {
    _information = information_ ;
    _sampler.setDistribution(_information.inverse());
  }

  const InformationType& information() {return _information; }

  virtual void sense() {
    _robotPoseObject = 0;
    // set the robot pose
    if (! robot())
      return;
    
    RobotType* r =dynamic_cast<RobotType*>(robot());
    if (!r)
      return;
    
    if(! r->trajectory().empty())
      _robotPoseObject = *(r->trajectory().rbegin());

    if (! world() || ! graph())
      return;
    
    // naive search. just for initial testing
    for(std::set<BaseWorldObject*>::iterator it=world()->objects().begin(); it!=world()->objects().end(); it++){
      WorldObjectType * wo = dynamic_cast<WorldObjectType*>(*it);
      if (wo){
  EdgeType* e=mkEdge(wo);
  if (e) {
    e->setMeasurementFromState();
    addNoise(e);
    graph()->addEdge(e);
  }
      }
    }
  }


 protected:
  PoseObject* _robotPoseObject;
  InformationType _information;

  EdgeType* mkEdge(WorldObjectType* object){
    PoseVertexType* robotVertex = (PoseVertexType*)_robotPoseObject->vertex();
    EdgeType* e = new EdgeType();
    e->vertices()[0]=robotVertex;
    e->vertices()[1]=object->vertex();
    e->information().setIdentity();
    return e;
  }
  GaussianSampler<typename EdgeType::ErrorVector, InformationType> _sampler;
  virtual void addNoise(EdgeType*){};
};

class G2O_SIMULATOR_API World{
 public:
  World(OptimizableGraph* graph_) {_graph = graph_; _runningId=0; _paramId=0;}
  OptimizableGraph* graph() {return _graph;}
  bool addRobot(BaseRobot* robot);
  bool addWorldObject(BaseWorldObject* worldObject);
  bool addParameter(Parameter* p);

  std::set<BaseWorldObject*>& objects() {return _objects;}
  std::set<BaseRobot*>&  robots() {return _robots; }
 protected:
  std::set<BaseWorldObject*> _objects;
  std::set<BaseRobot*> _robots;
  OptimizableGraph* _graph;
  int _runningId;
  int _paramId;
};

} // end namespace

#endif
