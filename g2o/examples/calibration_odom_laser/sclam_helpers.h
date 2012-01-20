#ifndef G2O_SCLAM_HELPERS_H
#define G2O_SCLAM_HELPERS_H

#include "g2o_calibration_odom_laser_api.h"

namespace g2o {

  class SparseOptimizer;
  class DataQueue;

  G2O_CALIBRATION_ODOM_LASER_API void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer, const DataQueue& odomData);

  G2O_CALIBRATION_ODOM_LASER_API void allocateSolverForSclam(SparseOptimizer& optimizer, bool levenberg = false);

} // end namespace

#endif
