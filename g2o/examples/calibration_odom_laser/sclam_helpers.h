#ifndef G2O_SCLAM_HELPERS_H
#define G2O_SCLAM_HELPERS_H

namespace g2o {

  class SparseOptimizer;
  class DataQueue;

  void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer, const DataQueue& odomData);

  void allocateSolverForSclam(SparseOptimizer& optimizer, bool levenberg = false);

} // end namespace

#endif
