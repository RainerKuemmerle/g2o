#ifndef G2O_SENSOR_POSE2D_H_
#define G2O_SENSOR_POSE2D_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API SensorPose2D : public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>  { 
  public:
    SensorPose2D(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);

    int stepsToIgnore() const {return _stepsToIgnore;}
    void setStepsToIgnore(int stepsToIgnore_) {_stepsToIgnore = stepsToIgnore_;}
  protected:
    bool isVisible(WorldObjectType* to);
    int _stepsToIgnore;
    // these are temporaries
    std::set<PoseObject*> _posesToIgnore;
  };

}

#endif
