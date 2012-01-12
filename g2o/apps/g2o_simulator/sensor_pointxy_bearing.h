#ifndef G2O_SENSOR_POINTXY_BEARING_H_
#define G2O_SENSOR_POINTXY_BEARING_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"

namespace g2o {

  class SensorPointXYBearing: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY>{ 
  public:
    SensorPointXYBearing(const std::string& name_);
    virtual void addNoise(EdgeType* e);
    virtual void sense();
  protected:
    bool isVisible(WorldObjectType* to);
  }; 

}

#endif
