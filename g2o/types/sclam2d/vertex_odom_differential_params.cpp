#include "vertex_odom_differential_params.h"

namespace g2o {

  VertexOdomDifferentialParams::VertexOdomDifferentialParams() :
    BaseVertex <3, Vector3d>()
  {
  }

  bool VertexOdomDifferentialParams::read(std::istream& is)
  {
    is >> _estimate(0) >> _estimate(1) >> _estimate(2);
    return true;
  }

  bool VertexOdomDifferentialParams::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2);
    return os.good();
  }

}
