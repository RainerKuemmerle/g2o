#include "sensor_pose3d.h"

namespace g2o {
  using namespace std;

  SensorPose3D::SensorPose3D(const std::string& name_): BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3>(name_){  
    _stepsToIgnore = 10;
    _information.setIdentity();
    _information*=100;
    _information(3,3)=10000;
    _information(4,4)=10000;
    _information(5,5)=1000;
    setInformation(_information);
  }

  void SensorPose3D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromMinimalVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

  bool SensorPose3D::isVisible(SensorPose3D::WorldObjectType* to){
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
  
 
  void SensorPose3D::sense() {
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
    graph()->addEdge(e);
    e->setMeasurementFromState();
          addNoise(e);
  }
      }
    }
  }

}
