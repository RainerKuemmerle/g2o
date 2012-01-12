#ifndef G2O_ODOMETRY_MEASUREMENT_H
#define G2O_ODOMETRY_MEASUREMENT_H

#include <Eigen/Core>

namespace g2o {

  /**
   * \brief velocity measurement of a differential robot
   */
  class VelocityMeasurement
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VelocityMeasurement();
      VelocityMeasurement(double vl, double vr, double dt);

      double vl() const { return _measurement(0);}
      void setVl(double v) { _measurement(0) = v;}

      double vr() const { return _measurement(1);}
      void setVr(double v) { _measurement(1) = v;}

      double dt() const { return _dt;}
      void setDt(double t) { _dt = t;}
      
      const Eigen::Vector2d& measurement() const { return _measurement;}

    protected:
      Eigen::Vector2d _measurement;
      double _dt;
  };

  /**
   * \brief A 2D odometry measurement expressed as a transformation
   */
  class MotionMeasurement
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      MotionMeasurement();
      MotionMeasurement(double x, double y, double theta, double dt);
      MotionMeasurement(const Eigen::Vector3d& m, double dt);

      double x() const { return _measurement(0);}
      void setX(double v) { _measurement(0) = v;}

      double y() const { return _measurement(1);}
      void setY(double v) { _measurement(1) = v;}

      double theta() const { return _measurement(2);}
      void setTheta(double v) { _measurement(2) = v;}

      double dt() const { return _dt;}
      void setDt(double t) { _dt = t;}

      const Eigen::Vector3d& measurement() const { return _measurement;}

    protected:
      Eigen::Vector3d _measurement;
      double _dt;
  };

  /**
   * \brief convert between the different types of odometry measurements
   */
  class OdomConvert
  {
    public:
      static VelocityMeasurement convertToVelocity(const MotionMeasurement& m);
      static MotionMeasurement convertToMotion(const VelocityMeasurement& vi, double l = 1.0);
  };

} // end namespace

#endif
