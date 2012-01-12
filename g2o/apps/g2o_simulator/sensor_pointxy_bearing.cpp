#include "sensor_pointxy_bearing.h"

namespace g2o {

  
  SensorPointXYBearing::SensorPointXYBearing(const std::string& name_): BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY>(name_) {
    _information(0,0)=1000;
  }

  void SensorPointXYBearing::addNoise(EdgeType* e){
    EdgeType::ErrorVector n=_sampler.generateSample();
    e->setMeasurement(e->measurement()+n(0));
    e->setInformation(information());
  }

  bool SensorPointXYBearing::isVisible(SensorPointXYBearing::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;
    
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _robotPoseObject->vertex()->estimate().inverse()*pose;
    Vector2d translation=delta;
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    double bearing=acos(translation.x());
    if (fabs(bearing)>_fov)
       return false;
    return true;
  }


  void SensorPointXYBearing::sense() {
    _robotPoseObject=0;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    int count = 0;
    while (it!=r->trajectory().rend() && count < 1){
      if (!_robotPoseObject)
  _robotPoseObject = *it;
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



}
