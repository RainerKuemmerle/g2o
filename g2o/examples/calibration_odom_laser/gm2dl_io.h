#include <string>

namespace g2o {

  class SparseOptimizer;
  class DataQueue;

  /**
   * \brief read / write gm2dl file into / out of a SparseOptimizer
   */
  class Gm2dlIO
  {
    public:
      /**
       * the global ID we assign the laser pose vertex
       */
      static const int ID_LASERPOSE;
      static const int ID_ODOMCALIB;

      static bool readGm2dl(const std::string& filename, SparseOptimizer& optimizer, bool overrideCovariances = false);

      static bool updateLaserData(SparseOptimizer& optimizer);

      static bool writeGm2dl(const std::string& filename, const SparseOptimizer& optimizer);

      static int readRobotLaser(const std::string& filename, DataQueue& queue);

  };

}
