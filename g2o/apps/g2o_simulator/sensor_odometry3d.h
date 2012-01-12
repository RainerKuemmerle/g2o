#ifndef G2O_SENSOR_ODOMETRY3D_H_
#define G2O_SENSOR_ODOMETRY3D_H_

#include "simulator3d_base.h"

namespace g2o {

  class SensorOdometry3D: public BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3 > {
  public:
    SensorOdometry3D(const std::string& name_);
    virtual void sense();
    void addNoise(EdgeType* e);
  };

}

#endif
