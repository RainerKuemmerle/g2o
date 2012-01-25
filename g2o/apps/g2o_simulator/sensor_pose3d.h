#ifndef G2O_SENSOR_POSE3D_H_
#define G2O_SENSOR_POSE3D_H_
#include "simulator3d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API SensorPose3D : public PointSensorParameters, public BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3>  { 
  public:
    SensorPose3D(const std::string& name_);
    virtual void sense();
    int stepsToIgnore() const {return _stepsToIgnore;}
    void setStepsToIgnore(int stepsToIgnore_) {_stepsToIgnore = stepsToIgnore_;}
    void addNoise(EdgeType* e);

  protected:
    
    bool isVisible(WorldObjectType* to);
    int _stepsToIgnore;
    // these are temporaries
    std::set<PoseObject*> _posesToIgnore;
  };

}

#endif
