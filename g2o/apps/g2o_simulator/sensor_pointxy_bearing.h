#ifndef G2O_SENSOR_POINTXY_BEARING_H_
#define G2O_SENSOR_POINTXY_BEARING_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API SensorPointXYBearing: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY>{ 
  public:
    SensorPointXYBearing(const std::string& name_);
    virtual void addNoise(EdgeType* e);
    virtual void sense();
  protected:
    bool isVisible(WorldObjectType* to);
  }; 

}

#endif
