#include <pybind11/pybind11.h>

#include "icp/types_icp.h"
#include "sba/sbacam.h"
#include "sba/types_sba.h"
#include "sba/types_six_dof_expmap.h"
#include "sclam2d/types_sclam2d.h"
#include "sim3/sim3.h"
#include "sim3/types_seven_dof_expmap.h"
#include "slam2d/types_slam2d.h"
#include "slam3d/types_slam3d.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareTypes(py::module& m) {
  // slam2d
  declareTypesSlam2d(m);

  // slam3d
  declareTypesSlam3d(m);

  // sba
  declareTypesSBA(m);
  declareTypesSixDofExpmap(m);
  declareSBACam(m);

  // sim3
  declareSim3(m);
  declareTypesSevenDofExpmap(m);

  // icp
  declareTypesICP(m);

  // sclam2d
  declareTypesSclam2d(m);
}

}  // end namespace g2o