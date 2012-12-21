#ifndef G2O_VERTEX_PLANE_H_
#define G2O_VERTEX_PLANE_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "line3d.h"

namespace Slam3dAddons {
  using namespace g2o;

  class VertexLine3D : public BaseVertex<6, Line3D>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexLine3D() {}
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() { _estimate = Line3D(); }

      virtual void oplusImpl(const double* update_) {
	Map<const Vector6d> update(update_);
	_estimate.oplus(update);
      }

      virtual bool setEstimateDataImpl(const double* est){
	Map<const Vector6d> _est(est);
	_estimate.fromVector(_est);
	return true;
      }

      virtual bool getEstimateData(double* est) const{
	Map<Vector6d> _est(est);
	_est = _estimate.toVector();
	return true;
      }

      virtual int estimateDimension() const {
	return 6;
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

  /* class VertexLine3DDrawAction: public DrawAction{ */
  /* public: */
  /*   VertexLine3DDrawAction(); */
  /*   virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
  /*           HyperGraphElementAction::Parameters* params_ ); */
  /* protected: */
  /*   virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_); */
  /*   FloatProperty* _planeWidth, *_planeHeight; */
  /* }; */

}
#endif
