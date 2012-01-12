#include "pointsensorparameters.h"

#include "g2o/stuff/misc.h" // for M_PI

// Robot2D
namespace g2o {
  PointSensorParameters::PointSensorParameters(){
    _maxRange2 = 25;
    _minRange2 = 0.01;
    _fov = M_PI/2;
    _maxAngularDifference = M_PI/2;
  }
}
