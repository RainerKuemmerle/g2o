#ifndef G2O_EDGE_SE3_PLANE_CALIB_H
#define G2O_EDGE_SE3_PLANE_CALIB_H

#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "vertex_plane.h"
#include "g2o/config.h"

namespace g2o {
  /**
   * \brief plane measurement that also calibrates an offset for the sensor
   */
  class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalib : public BaseMultiEdge<3, Plane3D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3PlaneSensorCalib();
      Vector3d color;

      void computeError()
      {
        const VertexSE3* v1            = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[1]);
        const VertexSE3* offset        = static_cast<const VertexSE3*>(_vertices[2]);
        const Plane3D& plane           = planeVertex->estimate();
	// measurement function: remap the plane in global coordinates
	Isometry3d w2n=(v1->estimate()*offset->estimate()).inverse();
	Plane3D localPlane=w2n*plane;
	_error = localPlane.ominus(_measurement);
      }

      void setMeasurement(const Plane3D& m){
	_measurement = m;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalibDrawAction: public DrawAction{
  public:
    EdgeSE3PlaneSensorCalibDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _planeWidth, *_planeHeight;
  };
#endif

} // end namespace

#endif
