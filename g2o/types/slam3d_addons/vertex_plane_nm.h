#ifndef G2O_VERTEX_PLANE_NM_H_
#define G2O_VERTEX_PLANE_NM_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "plane3d.h"

namespace Slam3dAddons {
  using namespace g2o;

  class VertexPlaneNM : public BaseVertex<4, Plane3D>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexPlaneNM() {}
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() { _estimate = Plane3D(); }

      virtual void oplusImpl(const double* update_) {
	Map<const Vector4d> update(update_);
	_estimate._coeffs += update;
	double k = 1./_estimate._coeffs.head<3>().norm();
	_estimate._coeffs *= k;
      }

      virtual bool setEstimateDataImpl(const double* est){
	Map<const Vector4d> _est(est);
	_estimate.fromVector(_est);
	return true;
      }

      virtual bool getEstimateData(double* est) const{
	Map<Vector4d> _est(est);
	_est = _estimate.toVector();
	return true;
      }

      virtual int estimateDimension() const {
	return 4;
      }

      /* virtual bool setMinimalEstimateDataImpl(const double* est){ */
      /*   Map<const Vector3d> est_(est); */
      /*   _estimate.fromMinimalVector(est_); */
      /*   return true; */
      /* } */

      /* virtual bool getMinimalEstimateData(double* est) const{ */
      /*   Map<Vector3d> v(est); */
      /*   v = _estimate.toMinimalVector(); */
      /*   return true; */
      /* } */

      /* virtual int minimalEstimateDimension() const { */
      /*   return 3; */
      /* } */

    };

  class VertexPlaneNMDrawAction: public DrawAction{
  public:
    VertexPlaneNMDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _planeWidth, *_planeHeight;
  };

}
#endif
