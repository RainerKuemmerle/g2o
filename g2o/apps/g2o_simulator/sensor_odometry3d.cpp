#include "sensor_odometry3d.h"

namespace g2o {
  using namespace std;


  SensorOdometry3D::SensorOdometry3D(const std::string& name_):
    BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3>(name_){
    _information.setIdentity();
    _information*=100;
    _information(3,3)=10000;
    _information(4,4)=10000;
    _information(5,5)=10000;
    setInformation(_information);
  }

  void SensorOdometry3D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromMinimalVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

  void SensorOdometry3D::sense(){
    
    if (! robot())
      return;
    
    RobotType* r =dynamic_cast<RobotType*>(robot());
    if (!r)
      return;
    
    PoseObject* pprev=0, *pcurr=0;
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    if (it!=r->trajectory().rend()){
      pcurr = *it; 
      it++;
    }
    if (it!=r->trajectory().rend()){
      pprev = *it; 
      it++;
    }
    if (!(pcurr&&pprev)) {
      cerr << __PRETTY_FUNCTION__ << ": fatal, trajectory empty" << endl;
      return;
    }
    _robotPoseObject = pprev;
    EdgeType* e=mkEdge(pcurr);
    if (e){
      if (graph()) {
    graph()->addEdge(e);
  e->setMeasurementFromState();
        addNoise(e);
      }
    }
    _robotPoseObject = pcurr;
  }

}
