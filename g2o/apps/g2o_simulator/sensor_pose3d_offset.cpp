#include "sensor_pose3d_offset.h"

namespace g2o {
  using namespace std;

  SensorPose3DOffset::SensorPose3DOffset(const std::string& name_): 
    BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3>(name_){  
    _offsetParam1 = _offsetParam2 =0;
    _stepsToIgnore = 10;
    _information.setIdentity();
    _information*=100;
    _information(3,3)=10000;
    _information(4,4)=10000;
    _information(5,5)=1000;
    setInformation(_information);
  }

  void SensorPose3DOffset::addParameters(){
    if (!_offsetParam1)
      _offsetParam1 = new ParameterSE3Offset();
    if (!_offsetParam2)
      _offsetParam2 = new ParameterSE3Offset();
    assert(world());
    world()->addParameter(_offsetParam1);
    world()->addParameter(_offsetParam2);
  }

  void SensorPose3DOffset::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromMinimalVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

  bool SensorPose3DOffset::isVisible(SensorPose3DOffset::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;
    if (_posesToIgnore.find(to)!=_posesToIgnore.end())
      return false;
    
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _robotPoseObject->vertex()->estimate().inverse()*pose;
    Vector3d translation=delta.translation();
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    double bearing=acos(translation.x());
    if (fabs(bearing)>_fov)
      return false;
    AngleAxisd a(delta.rotation());
    if (fabs(a.angle())>_maxAngularDifference)
      return false;
    return true;
  }
  
 
  void SensorPose3DOffset::sense() {
    _robotPoseObject=0;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    _posesToIgnore.clear();
    int count = 0;
    while (it!=r->trajectory().rend() && count < _stepsToIgnore){
      if (!_robotPoseObject)
  _robotPoseObject = *it;
      _posesToIgnore.insert(*it);
      it++;
      count++;
    }
    for (std::set<BaseWorldObject*>::iterator it=world()->objects().begin();
   it!=world()->objects().end(); it++){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(*it);
      if (o && isVisible(o)){
  EdgeType* e=mkEdge(o);  
  if (e && graph()) {
          e->setParameterId(0,_offsetParam1->id());
          e->setParameterId(1,_offsetParam2->id());
    graph()->addEdge(e);
    e->setMeasurementFromState();
          addNoise(e);
  }
      }
    }
  }

}
