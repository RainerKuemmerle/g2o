#ifndef G2O_SENSOR_POINTXY_H_
#define G2O_SENSOR_POINTXY_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"

namespace g2o {
  
  class SensorPointXY: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2PointXY, WorldObjectPointXY>{ 
  public:
    SensorPointXY(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);
  protected:
    bool isVisible(WorldObjectType* to);
  }; 

}

#endif
