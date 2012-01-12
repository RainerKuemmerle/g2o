#ifndef G2O_SIMULATOR3D_BASE_H_
#define G2O_SIMULATOR3D_BASE_H_

#include "g2o/types/slam3d/types_slam3d.h"
#include "simulator.h"

namespace g2o {

  typedef WorldObject<VertexSE3> WorldObjectSE3;

  typedef WorldObject<VertexPointXYZ> WorldObjectTrackXYZ;

  typedef Robot<WorldObjectSE3>  Robot3D;

}

#endif
