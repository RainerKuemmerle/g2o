#ifndef G2O_VERTEX_ODOM_DIFFERENTIAL_PARAMS_H
#define G2O_VERTEX_ODOM_DIFFERENTIAL_PARAMS_H

#include "g2o_types_sclam2d_api.h"
#include "g2o/core/base_vertex.h"

namespace g2o {

  class G2O_TYPES_SCLAM2D_API VertexOdomDifferentialParams: public BaseVertex <3, Vector3d> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexOdomDifferentialParams();
      virtual void setToOriginImpl() {
        _estimate << 1. , 1., 1.;
      }

      virtual void oplusImpl(const double* v) {
        for (int i=0; i<3; i++)
          _estimate(i) += v[i];
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

}

#endif
