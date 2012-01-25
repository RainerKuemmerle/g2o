#ifndef G2O_SENSOR_ODOMETRY3D_H_
#define G2O_SENSOR_ODOMETRY3D_H_

#include "simulator3d_base.h"
#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API SensorOdometry3D: public BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3 > {
  public:
    SensorOdometry3D(const std::string& name_);
    virtual void sense();
    void addNoise(EdgeType* e);
  };

}

#endif
