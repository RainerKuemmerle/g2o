// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "types_slam2d.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

namespace types_slam2d {

  G2O_REGISTER_TYPE(VERTEX_SE2, VertexSE2);
  G2O_REGISTER_TYPE(VERTEX_XY, VertexPointXY);
  G2O_REGISTER_TYPE(PARAMS_SE2OFFSET, ParameterSE2Offset);
  G2O_REGISTER_TYPE(CACHE_SE2_OFFSET, CacheSE2Offset);
  G2O_REGISTER_TYPE(EDGE_PRIOR_SE2, EdgeSE2Prior);
  G2O_REGISTER_TYPE(EDGE_SE2, EdgeSE2);
  G2O_REGISTER_TYPE(EDGE_SE2_XY, EdgeSE2PointXY);
  G2O_REGISTER_TYPE(EDGE_BEARING_SE2_XY, EdgeSE2PointXYBearing);
  G2O_REGISTER_TYPE(EDGE_SE2_XY_CALIB, EdgeSE2PointXYCalib);
  G2O_REGISTER_TYPE(EDGE_SE2_OFFSET, EdgeSE2Offset);
  G2O_REGISTER_TYPE(EDGE_SE2_POINTXY_OFFSET, EdgeSE2PointXYOffset);

  G2O_REGISTER_ACTION(VertexSE2WriteGnuplotAction);
  G2O_REGISTER_ACTION(VertexPointXYWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2WriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYBearingWriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(VertexSE2DrawAction);
  G2O_REGISTER_ACTION(VertexPointXYDrawAction);
  G2O_REGISTER_ACTION(EdgeSE2DrawAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYDrawAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYBearingDrawAction);
#endif
}
} // end namespace


