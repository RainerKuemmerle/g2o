#include "sensor_pose2d.h"

namespace g2o{
  SensorPose2D::SensorPose2D(const std::string& name_):
    BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_)
  {
    _stepsToIgnore = 10;
  }

  bool SensorPose2D::isVisible(SensorPose2D::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;
    if (_posesToIgnore.find(to)!=_posesToIgnore.end())
      return false;
    
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _robotPoseObject->vertex()->estimate().inverse()*pose;
    Vector2d translation=delta.translation();
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    double bearing=acos(translation.x());
    if (fabs(bearing)>_fov)
      return false;
    if (fabs(delta.rotation().angle())>_maxAngularDifference)
      return false;
    return true;
  }
  

  void SensorPose2D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }
 
  void SensorPose2D::sense() {
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
	  e->setMeasurementFromState();
	  addNoise(e);
	  graph()->addEdge(e);
	}
      }
    }
  }

} // end namespace
