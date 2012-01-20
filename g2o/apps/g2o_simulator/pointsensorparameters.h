#ifndef G2O_POINTSENSORPARAMETERS_H_
#define G2O_POINTSENSORPARAMETERS_H_

#include <cmath>

#include "g2o_simulator_api.h"

namespace g2o {

  class G2O_SIMULATOR_API PointSensorParameters{
  public:
    PointSensorParameters();
    double maxRange() const {return sqrt(_maxRange2);}
    void setMaxRange(double maxRange_)  {_maxRange2 = maxRange_*maxRange_;}
    double minRange() const {return sqrt(_minRange2);}
    void setMinRange(double minRange_)  {_minRange2 = minRange_*minRange_;}
    double fov() const {return _fov;}
    void setFov(double fov_)  {_fov = fov_;}
    double maxAngularDifference() const { return _maxAngularDifference; }
    void setMaxAngularDifference(double angularDifference) {_maxAngularDifference = angularDifference;}
  protected:
    double _maxRange2, _minRange2, _fov,  _maxAngularDifference;
  };

}

#endif
