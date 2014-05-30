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

#include "types_slam3d_addons.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <typeinfo>
#include <iostream>

namespace g2o {

  G2O_REGISTER_TYPE_GROUP(slam3d_addons);

  G2O_REGISTER_TYPE(VERTEX3, VertexSE3Euler);
  G2O_REGISTER_TYPE(EDGE3, EdgeSE3Euler);
  G2O_REGISTER_TYPE(VERTEX_PLANE, VertexPlane);
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE_CALIB, EdgeSE3PlaneSensorCalib);

  G2O_REGISTER_TYPE(VERTEX_LINE3D, VertexLine3D);
  G2O_REGISTER_TYPE(EDGE_SE3_LINE3D, EdgeSE3Line3D);
  G2O_REGISTER_TYPE(EDGE_LINE3D, EdgeLine3D);
  G2O_REGISTER_TYPE(EDGE_PLANE, EdgePlane);
  G2O_REGISTER_TYPE(EDGE_SE3_CALIB, EdgeSE3Calib);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(CacheCameraDrawAction);
  G2O_REGISTER_ACTION(VertexPlaneDrawAction);
  G2O_REGISTER_ACTION(EdgeSE3PlaneSensorCalibDrawAction);
#endif

  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_addons_types)
  {
    static bool initialized = false;
    if (initialized)
      return;
    initialized = true;

#ifdef G2O_HAVE_OPENGL
    HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
    HyperGraphElementAction* vertexse3eulerdraw=new g2o::VertexSE3DrawAction;
    vertexse3eulerdraw->setTypeName(typeid(VertexSE3Euler).name());
    actionLib->registerAction(vertexse3eulerdraw);

    HyperGraphElementAction* edgese3eulerdraw=new g2o::EdgeSE3DrawAction;
    edgese3eulerdraw->setTypeName(typeid(EdgeSE3Euler).name());
    actionLib->registerAction(edgese3eulerdraw);
#endif
  }

} // end namespace
