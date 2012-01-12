#include "sensor_pointxyz.h"

namespace g2o {
  using namespace std;

  // SensorPointXYZ
  SensorPointXYZ::SensorPointXYZ(const std::string& name_): BinarySensor<Robot3D, EdgeSE3PointXYZ, WorldObjectTrackXYZ>(name_) {
    _offsetParam = 0;
    _information.setIdentity();
    _information*=1000;
    _information(2,2)=10;
    setInformation(_information);
  }

  bool SensorPointXYZ::isVisible(SensorPointXYZ::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _sensorPose.inverse()*pose;
    Vector3d translation=delta;
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    // the cameras have the z in front
    double bearing=acos(translation.z());
    if (fabs(bearing)>_fov)
       return false;
    return true;
  }

  void SensorPointXYZ::addParameters(){
    if (!_offsetParam)
      _offsetParam = new ParameterSE3Offset();
    assert(world());
    world()->addParameter(_offsetParam);
  }

  void SensorPointXYZ::addNoise(EdgeType* e){
    EdgeType::ErrorVector n=_sampler.generateSample();
    e->setMeasurement(e->measurement()+n);
    e->setInformation(information());
  }

  void SensorPointXYZ::sense() {
    if (! _offsetParam){
      return;
    }
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
    if (!_robotPoseObject)
      return;
    _sensorPose = _robotPoseObject->vertex()->estimate()*_offsetParam->offset();
    for (std::set<BaseWorldObject*>::iterator it=world()->objects().begin();
   it!=world()->objects().end(); it++){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(*it);
      if (o && isVisible(o)){
  EdgeType* e=mkEdge(o);
  e->setParameterId(0,_offsetParam->id());
  if (e && graph()) {
    graph()->addEdge(e);
    e->setMeasurementFromState();
          addNoise(e);
  }
      }
    }
  }

}
