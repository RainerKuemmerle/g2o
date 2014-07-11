#ifndef G2O_VERTEX_LINE3D_H_
#define G2O_VERTEX_LINE3D_H_

#include "g2o_types_slam3d_addons_api.h"
#include "line3d.h"

#include "g2o/core/base_vertex.h"

namespace g2o {

  class G2O_TYPES_SLAM3D_ADDONS_API VertexLine3D : public BaseVertex<6, Line3D>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexLine3D() {}
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() { _estimate = Line3D(); }

      virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);
	_estimate.oplus(update);
      }

      virtual bool setEstimateDataImpl(const double* est){
        Eigen::Map<const Vector6d> _est(est);
	_estimate=Line3D(_est);
	return true;
      }

      virtual bool getEstimateData(double* est) const{
        Eigen::Map<Vector6d> _est(est);
	_est = _estimate;
	return true;
      }

      virtual int estimateDimension() const {
	return 6;
      }

    };

}
#endif
