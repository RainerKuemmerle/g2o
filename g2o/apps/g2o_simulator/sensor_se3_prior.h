#ifndef G2O_SENSOR_SE3_PRIOR_H_
#define G2O_SENSOR_SE3_PRIOR_H_
#include "simulator3d_base.h"
#include "pointsensorparameters.h"

namespace g2o {

  class SensorSE3Prior: public PointSensorParameters, public UnarySensor<Robot3D, EdgeSE3Prior>{
  public:
    typedef PoseVertexType::EstimateType RobotPoseType;
    SensorSE3Prior(const std::string& name_);
    virtual void sense();
    virtual void addParameters();
    ParameterSE3Offset* offsetParam() {return _offsetParam;};
    void addNoise(EdgeType* e);
  protected:
    RobotPoseType _sensorPose;
    ParameterSE3Offset* _offsetParam;
  };

}

#endif
