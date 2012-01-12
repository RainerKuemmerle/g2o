#ifndef G2O_SENSOR_POINTXYZ_H_
#define G2O_SENSOR_POINTXYZ_H_
#include "simulator3d_base.h"
#include "pointsensorparameters.h"

namespace g2o {

  class SensorPointXYZ: public PointSensorParameters, public BinarySensor<Robot3D, EdgeSE3PointXYZ, WorldObjectTrackXYZ>{
  public:
    typedef PoseVertexType::EstimateType RobotPoseType;
    SensorPointXYZ(const std::string& name_);
    virtual void sense();
    virtual void addParameters();
    ParameterSE3Offset* offsetParam() {return _offsetParam;};
    void addNoise(EdgeType* e);
  protected:
    bool isVisible(WorldObjectType* to);
    RobotPoseType _sensorPose;
    ParameterSE3Offset* _offsetParam;
  };

}

#endif
