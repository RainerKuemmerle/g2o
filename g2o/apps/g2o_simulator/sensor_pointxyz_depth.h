#ifndef G2O_SENSOR_POINTXYZ_DEPTH_H_
#define G2O_SENSOR_POINTXYZ_DEPTH_H_
#include "simulator3d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API SensorPointXYZDepth: public PointSensorParameters, public BinarySensor<Robot3D, EdgeSE3PointXYZDepth, WorldObjectTrackXYZ>{
  public:
    typedef PoseVertexType::EstimateType RobotPoseType;
    SensorPointXYZDepth(const std::string& name_);
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
