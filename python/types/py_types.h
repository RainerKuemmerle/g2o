#pragma once

#include "g2opy.h"
#include "icp/py_types_icp.h"
#include "pure/py_types_pure.h"
#include "sba/py_sbacam.h"
#include "sba/py_types_sba.h"
#include "sba/py_types_six_dof_expmap.h"
#include "sclam2d/py_types_sclam2d.h"
#include "sim3/py_types_seven_dof_expmap.h"
#include "slam2d/py_types_slam2d.h"
#include "slam3d/py_types_slam3d.h"

namespace g2o {

inline void declareTypes(py::module& m) {
  // slam2d
  declareTypesSlam2d(m);

  // slam3d
  declareTypesSlam3d(m);

  // sba
  declareTypesSBA(m);
  declareTypesSixDofExpmap(m);
  declareSBACam(m);

  // sim3
  declareTypesSevenDofExpmap(m);

  // icp
  declareTypesICP(m);

  // sclam2d
  declareTypesSclam2d(m);

  // pure python types
  declareTypesPure(m);
}

}  // end namespace g2o
