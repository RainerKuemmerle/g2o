#ifndef G2O_VERTEX_LINE3D_H_
#define G2O_VERTEX_LINE3D_H_

#include "g2o/core/base_vertex.h"
#include "line3d.h"

namespace g2o {

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
	_estimate=Line3D(_est);
	return true;
      }

      virtual bool getEstimateData(double* est) const{
	Map<Vector6d> _est(est);
	_est = _estimate;
	return true;
      }

      virtual int estimateDimension() const {
	return 6;
      }

    };

}
#endif
