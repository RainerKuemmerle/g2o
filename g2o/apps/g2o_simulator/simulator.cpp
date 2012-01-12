#include "simulator.h"
#include <iostream>
namespace g2o{
  using namespace std;

  // BaseWorldObject
  BaseWorldObject::~BaseWorldObject() {}
  OptimizableGraph* BaseWorldObject::graph() {
    if (_world)
      return _world-> graph();
    return 0;
  }

  void BaseWorldObject::setVertex(OptimizableGraph::Vertex* vertex_){ 
    _vertex= vertex_; 
  }

  // BaseRobot
  OptimizableGraph* BaseRobot::graph() {
    if (_world)
      return _world-> graph();
    return 0;
  }

  bool BaseRobot::addSensor(BaseSensor* sensor){
    assert (graph());
    std::pair<std::set<BaseSensor*>::iterator, bool> result=_sensors.insert(sensor);
    if(result.second){
      sensor->setRobot(this);
      sensor->addParameters();
    }
    return result.second;
  }

  void BaseRobot::sense() {
    for (std::set<BaseSensor*>::iterator it=_sensors.begin(); it!=_sensors.end(); it++){
      BaseSensor* s=*it;
      s->sense();
    }
  }

  // Sensor
  World* BaseSensor::world() {
    if (!_robot)
      return 0;
    return _robot->world();
  }

  OptimizableGraph* BaseSensor::graph() {
    if (!_robot)
      return 0;
    return _robot->graph();
  }

  //World
  bool World::addRobot(BaseRobot* robot){
    std::pair<std::set<BaseRobot*>::iterator, bool> result=_robots.insert(robot);
    if (result.second){
      robot->setWorld(this);
    }
    return result.second;
  }
  
  bool World::addWorldObject(BaseWorldObject* object){
    std::pair<std::set<BaseWorldObject*>::iterator, bool> result=_objects.insert(object);
    if (result.second){
      object->setWorld(this);
    }
    if (graph() && object->vertex()){
      object->vertex()->setId(_runningId++);
      graph()->addVertex(object->vertex());
    }
    return result.second;
  }

  bool World::addParameter(Parameter* param){
    if ( !graph())
      return false;
    param->setId(_paramId);
    graph()->addParameter(param);
    _paramId++;
    return true;
  }


} // end namespace
