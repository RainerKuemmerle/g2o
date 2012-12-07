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

#include <iostream>

namespace Slam3dAddons {
  int initialized = 0;
}

namespace g2o {
  using namespace std;


  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_addons_types)
  {
    if (Slam3dAddons::initialized)
      return;
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;
    factory->registerType("VERTEX3", new HyperGraphElementCreator< ::Slam3dAddons::VertexSE3Euler>);
    factory->registerType("EDGE3", new HyperGraphElementCreator< ::Slam3dAddons::EdgeSE3Euler>);
    factory->registerType("VERTEX_PLANE", new HyperGraphElementCreator< ::Slam3dAddons::VertexPlane>);
    factory->registerType("EDGE_SE3_PLANE_CALIB", new HyperGraphElementCreator< ::Slam3dAddons::EdgeSE3PlaneSensorCalib>);

    HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
#ifdef G2O_HAVE_OPENGL
    HyperGraphElementAction* vertexse3eulerdraw=new g2o::VertexSE3DrawAction;
    vertexse3eulerdraw->setTypeName(typeid( ::Slam3dAddons::VertexSE3Euler).name());
    actionLib->registerAction(vertexse3eulerdraw);

    HyperGraphElementAction* edgese3eulerdraw=new g2o::EdgeSE3DrawAction;
    edgese3eulerdraw->setTypeName(typeid( ::Slam3dAddons::EdgeSE3Euler).name());
    actionLib->registerAction(edgese3eulerdraw);


    actionLib->registerAction(new  ::Slam3dAddons::CacheCameraDrawAction);
    actionLib->registerAction(new  ::Slam3dAddons::VertexPlaneDrawAction);
#endif
    ::Slam3dAddons::initialized = 1;
  }

} // end namespace
