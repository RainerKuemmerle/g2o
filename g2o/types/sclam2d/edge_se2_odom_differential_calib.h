#ifndef G2O_EDGE_SE2_ODOM_CALIB_DIFFERENTIAL_H
#define G2O_EDGE_SE2_ODOM_CALIB_DIFFERENTIAL_H

#include "g2o_types_sclam2d_api.h"
#include "odometry_measurement.h"
#include "vertex_odom_differential_params.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/base_multi_edge.h"

namespace g2o {

  class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalib : public BaseMultiEdge<3, VelocityMeasurement>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2OdomDifferentialCalib();

      void computeError()
      {
        const VertexSE2* v1                        = dynamic_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2                        = dynamic_cast<const VertexSE2*>(_vertices[1]);
        const VertexOdomDifferentialParams* params = dynamic_cast<const VertexOdomDifferentialParams*>(_vertices[2]);
        const SE2& x1                              = v1->estimate();
        const SE2& x2                              = v2->estimate();

        // get the calibrated motion given by the odometry
        VelocityMeasurement calibratedVelocityMeasurment(measurement().vl() * params->estimate()(0),
            measurement().vr() * params->estimate()(1),
            measurement().dt());
        MotionMeasurement mm = OdomConvert::convertToMotion(calibratedVelocityMeasurment, params->estimate()(2));
        SE2 Ku_ij;
        Ku_ij.fromVector(mm.measurement());

        SE2 delta = Ku_ij.inverse() * x1.inverse() * x2 ;
        _error = delta.toVector();
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalibDrawAction: public DrawAction {
    public:
      EdgeSE2OdomDifferentialCalibDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
