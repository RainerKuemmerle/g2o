#ifndef G2O_SIMULATOR2D_BASE_H_
#define G2O_SIMULATOR2D_BASE_H_

#include "g2o/types/slam2d/types_slam2d.h"
#include "simulator.h"

namespace g2o {

  typedef WorldObject<VertexSE2> WorldObjectSE2;

  typedef WorldObject<VertexPointXY> WorldObjectPointXY;

  typedef Robot<WorldObjectSE2>  Robot2D;

}

#endif
