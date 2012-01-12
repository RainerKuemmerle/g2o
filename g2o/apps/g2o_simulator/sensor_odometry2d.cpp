#include <cassert>
#include <iostream>
#include "sensor_odometry2d.h"

// Robot2D
namespace g2o {
  using namespace std;


  SensorOdometry2D::SensorOdometry2D(const std::string& name_):
    BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_){}

  void SensorOdometry2D::sense(){
    
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
      e->setMeasurementFromState();
      addNoise(e);
      if (graph())
    graph()->addEdge(e);
    }
    _robotPoseObject = pcurr;
  }
  
  void SensorOdometry2D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

}//
