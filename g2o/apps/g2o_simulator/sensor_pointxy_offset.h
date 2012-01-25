#ifndef G2O_SENSOR_POINTXY_OFFSET_H_
#define G2O_SENSOR_POINTXY_OFFSET_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {
  
  class G2O_SIMULATOR_API SensorPointXYOffset: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2PointXYOffset, WorldObjectPointXY>{ 
  public:
    typedef PoseVertexType::EstimateType RobotPoseType;
    SensorPointXYOffset(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);
    virtual void addParameters();
  protected:
    bool isVisible(WorldObjectType* to);
    ParameterSE2Offset* _offsetParam;
    RobotPoseType _sensorPose;
  }; 

}

#endif
