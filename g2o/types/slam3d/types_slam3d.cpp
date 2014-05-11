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

#include "types_slam3d.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

  G2O_REGISTER_TYPE_GROUP(slam3d);

  G2O_REGISTER_TYPE(VERTEX_SE3:QUAT, VertexSE3);
  G2O_REGISTER_TYPE(EDGE_SE3:QUAT, EdgeSE3);
  G2O_REGISTER_TYPE(VERTEX_TRACKXYZ, VertexPointXYZ);

  G2O_REGISTER_TYPE(PARAMS_SE3OFFSET, ParameterSE3Offset);
  G2O_REGISTER_TYPE(EDGE_SE3_TRACKXYZ, EdgeSE3PointXYZ);
  G2O_REGISTER_TYPE(EDGE_SE3_PRIOR, EdgeSE3Prior);
  G2O_REGISTER_TYPE(CACHE_SE3_OFFSET, CacheSE3Offset);
  G2O_REGISTER_TYPE(EDGE_SE3_OFFSET, EdgeSE3Offset);

  G2O_REGISTER_TYPE(PARAMS_CAMERACALIB, ParameterCamera);
  G2O_REGISTER_TYPE(PARAMS_STEREOCAMERACALIB, ParameterStereoCamera);
  G2O_REGISTER_TYPE(CACHE_CAMERA, CacheCamera);
  G2O_REGISTER_TYPE(EDGE_PROJECT_DISPARITY, EdgeSE3PointXYZDisparity);
  G2O_REGISTER_TYPE(EDGE_PROJECT_DEPTH, EdgeSE3PointXYZDepth);

  G2O_REGISTER_TYPE(EDGE_POINTXYZ, EdgePointXYZ);

  G2O_REGISTER_TYPE(EDGE_SE3_LOTSOF_XYZ, EdgeSE3LotsOfXYZ)

  /*********** ACTIONS ************/
  G2O_REGISTER_ACTION(VertexSE3WriteGnuplotAction);
  G2O_REGISTER_ACTION(VertexPointXYZWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE3WriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(VertexPointXYZDrawAction);
  G2O_REGISTER_ACTION(VertexSE3DrawAction);
  G2O_REGISTER_ACTION(EdgeSE3DrawAction);
  G2O_REGISTER_ACTION(EdgeSE3PointXYZDrawAction);
  G2O_REGISTER_ACTION(EdgeProjectDisparityDrawAction);
  G2O_REGISTER_ACTION(CacheCameraDrawAction);
  G2O_REGISTER_ACTION(CacheSE3OffsetDrawAction);
#endif

} // end namespace
