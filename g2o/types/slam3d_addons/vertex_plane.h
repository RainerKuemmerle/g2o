#ifndef G2O_VERTEX_PLANE_H_
#define G2O_VERTEX_PLANE_H_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "plane3d.h"

namespace g2o
{

  class VertexPlane : public BaseVertex<3, Plane3D>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexPlane();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() { _estimate = Plane3D(); }

      virtual void oplusImpl(const double* update_) {
	Map<const Vector3d> update(update_);
	_estimate.oplus(update);
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

      Vector3d color;
    };

#ifdef G2O_HAVE_OPENGL
  class VertexPlaneDrawAction: public DrawAction
  {
    public:
      VertexPlaneDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
          HyperGraphElementAction::Parameters* params_ );
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _planeWidth, *_planeHeight;
  };
#endif

}
#endif
