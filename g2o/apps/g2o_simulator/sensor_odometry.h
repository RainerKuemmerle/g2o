#ifndef G2O_ODOMETRY_SENSOR_
#define G2O_ODOMETRY_SENSOR_

#include "simulator.h"
namespace g2o {

  template <class  R, class  E, class O>
  class SensorOdometry: public BinarySensor<R, E, O > {
  public:
  SensorOdometry(const std::string name_): BinarySensor<R, E, O> (name_){};
    virtual void sense(){
      if (!  BinarySensor<R, E, O>::robot())
  return;
      
      typename BinarySensor<R, E, O>::RobotType* r =dynamic_cast<typename BinarySensor<R, E, O>::RobotType*>(robot());
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
  addNoise(e);
  if (graph())
    graph()->addEdge(e);
      }
    _robotPoseObject = pcurr;
    
    }
  };

}
#endif

