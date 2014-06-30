#ifndef G2O_EDGE_SE3_LINE_H_
#define G2O_EDGE_SE3_LINE_H_

#include "g2o/core/base_binary_edge.h"

#include "g2o_types_slam3d_addons_api.h"
#include "line3d.h"
#include "vertex_line3d.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

namespace g2o {

  typedef Eigen::Matrix<double, 7, 1> Vector7d;

  /**
   * TODO this documentation is totally wrong... copy and paste
   * \brief Edge between two 3D pose vertices
   *
   * The transformation between the two vertices is given as an Isometry3d.
   * If z denotes the measurement, then the error function is given as follows:
   * z^-1 * (x_i^-1 * x_j)
   */
  class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Line3D : public BaseBinaryEdge<7, Vector7d, VertexSE3, VertexLine3D> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Line3D();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Line3D& m){
        _measurement.head<6>() = Line3D(m);
	_measurement(6) = 1;
      }

      virtual bool setMeasurementData(const double* d){
        Map<const Vector7d> v(d);
        _measurement = v;
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Map<Vector7d> v(d);
        v = _measurement;
        return true;
      }

      virtual int measurementDimension() const {return 7;}

  private:

    ParameterSE3Offset* offsetParam;
    CacheSE3Offset* cache;
    virtual bool resolveCaches();

  };

} // end namespace
#endif
