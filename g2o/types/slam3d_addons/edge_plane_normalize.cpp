#include "edge_plane_normalize.h"

namespace Slam3dAddons {

  EdgePlaneNormalize::EdgePlaneNormalize(){
    _measurement = 1;
  }
  
  bool EdgePlaneNormalize::read(std::istream& is){
    is >> _measurement;
    return is.good();
  }

  bool EdgePlaneNormalize::write(std::ostream& os) const {
    os << _measurement;
    return os.good();
 }

}
