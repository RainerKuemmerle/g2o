#pragma once

#include "detail/registry.h"
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

inline void declareTypes(detail::Registry& registry) {
  // slam2d
  declareTypesSlam2d(registry);

  // slam3d
  declareTypesSlam3d(registry);

  // sba
  declareSBACam(registry.mod());
  declareTypesSBA(registry);
  declareTypesSixDofExpmap(registry);

  // sim3
  declareTypesSevenDofExpmap(registry);

  // icp
  declareTypesICP(registry);

  // sclam2d
  declareTypesSclam2d(registry);

  // pure python types
  declareTypesPure(registry);
}

}  // end namespace g2o
