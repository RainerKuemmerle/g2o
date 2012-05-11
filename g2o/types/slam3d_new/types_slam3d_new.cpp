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

#include "types_slam3d_new.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

#include "types_slam3d_new.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace Slam3dNew {
  int initialized = 0;
}

namespace g2o {
  using namespace std;

  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_types)
  {
    if (Slam3dNew::initialized)
      return;
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;
    factory->registerType("VERTEX_SE3_NEW", new HyperGraphElementCreator< ::Slam3dNew::VertexSE3>);
    factory->registerType("EDGE_SE3_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3>);
    factory->registerType("VERTEX_TRACKXYZ_NEW", new HyperGraphElementCreator< ::Slam3dNew::VertexPointXYZ>);

    factory->registerType("PARAMS_SE3OFFSET_NEW", new HyperGraphElementCreator< ::Slam3dNew::ParameterSE3Offset>);
    factory->registerType("EDGE_SE3_TRACKXYZ_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3PointXYZ>);
    factory->registerType("EDGE_SE3_PRIOR_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3Prior>);
    factory->registerType("CACHE_SE3_OFFSET_NEW", new HyperGraphElementCreator< ::Slam3dNew::CacheSE3Offset>);
    factory->registerType("EDGE_SE3_OFFSET_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3Offset>);

    factory->registerType("PARAMS_CAMERACALIB_NEW", new HyperGraphElementCreator< ::Slam3dNew::ParameterCamera>);
    factory->registerType("CACHE_CAMERA_NEW", new HyperGraphElementCreator< ::Slam3dNew::CacheCamera>);
    factory->registerType("EDGE_PROJECT_DISPARITY_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3PointXYZDisparity>);    
    factory->registerType("EDGE_PROJECT_DEPTH_NEW", new HyperGraphElementCreator< ::Slam3dNew::EdgeSE3PointXYZDepth>);
    

    HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
    actionLib->registerAction(new ::Slam3dNew::VertexSE3WriteGnuplotAction);
    actionLib->registerAction(new ::Slam3dNew::EdgeSE3WriteGnuplotAction);
    
#ifdef G2O_HAVE_OPENGL
    actionLib->registerAction(new ::Slam3dNew::VertexPointXYZDrawAction);
    actionLib->registerAction(new ::Slam3dNew::VertexSE3DrawAction);
    actionLib->registerAction(new ::Slam3dNew::EdgeSE3DrawAction);
    actionLib->registerAction(new ::Slam3dNew::CacheCameraDrawAction);
#endif
    ::Slam3dNew::initialized = 1;
  }

} // end namespace
