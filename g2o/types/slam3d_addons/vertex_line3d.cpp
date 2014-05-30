#include "vertex_line3d.h"

namespace g2o {

  bool VertexLine3D::read(std::istream& is) {
    Vector6d lv;
    for (int i=0; i<6; i++)
      is >> lv[i];
    setEstimate(Line3D(lv));
    return true;
  }

  bool VertexLine3D::write(std::ostream& os) const {
    Vector6d lv=_estimate;
    for (int i=0; i<6; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

}
