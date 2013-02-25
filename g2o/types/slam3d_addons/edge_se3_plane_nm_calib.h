#ifndef G2O_EDGE_SE3_PLANE_NM_CALIB_H
#define G2O_EDGE_SE3_PLANE_NM_CALIB_H

#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "vertex_plane_nm.h"

namespace Slam3dAddons {
  using namespace g2o;
  /**
   * \brief plane measurement that also calibrates an offset for the sensor
   */
  class EdgeSE3PlaneNMSensorCalib : public BaseMultiEdge<4, Vector4d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3PlaneNMSensorCalib();

      void computeError()
      {
        const VertexSE3* v1       = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexPlaneNM* planeVertex     = static_cast<const VertexPlaneNM*>(_vertices[1]);
        const VertexSE3* offset = static_cast<const VertexSE3*>(_vertices[2]);
        const Plane3D& plane = planeVertex->estimate();
	// measurement function: remap the plane in global coordinates
	Isometry3d w2n=(v1->estimate()*offset->estimate()).inverse();
	
	Vector4d v=plane._coeffs;
	Vector4d v2;
	Matrix3d R=w2n.rotation();
	v2.head<3>() = R*v.head<3>();
	v2(3)=v(3) - w2n.translation().dot(v2.head<3>());
	_error = v2 - _measurement;
      }

      void setMeasurement(const Vector4d& m){
	_measurement = m;
      }

      /* virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) */
      /* { */
      /*   if (   from.count(_vertices[2]) == 1 // need the laser offset */
      /*       && ((from.count(_vertices[0]) == 1 && to == _vertices[1]) || ((from.count(_vertices[1]) == 1 && to == _vertices[0])))) { */
      /*     return 1.0; */
      /*   } */
      /*   return -1.0; */
      /* } */
      /* virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to); */

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };


  class EdgeSE3PlaneNMSensorCalibDrawAction: public DrawAction{
  public:
    EdgeSE3PlaneNMSensorCalibDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _planeWidth, *_planeHeight;
  };

} // end namespace

#endif
