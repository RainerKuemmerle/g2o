#include "sensor_se3_prior.h"

namespace g2o {
  using namespace std;

  // SensorSE3Prior
  SensorSE3Prior::SensorSE3Prior(const std::string& name_): UnarySensor<Robot3D, EdgeSE3Prior>(name_) {
    _offsetParam = 0;
    _information.setIdentity();
    _information*=1000;
    _information(2,2)=10;
    setInformation(_information);
  }


  void SensorSE3Prior::addParameters(){
    if (!_offsetParam)
      _offsetParam = new ParameterSE3Offset();
    assert(world());
    world()->addParameter(_offsetParam);
  }

  void SensorSE3Prior::addNoise(EdgeType* e){
    EdgeType::ErrorVector _n=_sampler.generateSample();
    SE3Quat n;
    n.fromMinimalVector(_n);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

  void SensorSE3Prior::sense() {
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
    EdgeType* e=mkEdge();
    e->setParameterId(0,_offsetParam->id());
    if (e && graph()) {
      graph()->addEdge(e);
      e->setMeasurementFromState();
      addNoise(e);
    }
  }

}
