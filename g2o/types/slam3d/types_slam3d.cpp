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
  using namespace std;

  namespace types_slam3d {
    int initialized = 0;

    void init()
    {
      if (types_slam3d::initialized)
        return;
      Factory* factory = Factory::instance();
      //std::cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << std::endl;
      factory->registerType("VERTEX_SE3:QUAT", new HyperGraphElementCreator<VertexSE3>);
      factory->registerType("EDGE_SE3:QUAT", new HyperGraphElementCreator<EdgeSE3>);
      factory->registerType("VERTEX_TRACKXYZ", new HyperGraphElementCreator<VertexPointXYZ>);

      factory->registerType("PARAMS_SE3OFFSET", new HyperGraphElementCreator<ParameterSE3Offset>);
      factory->registerType("EDGE_SE3_TRACKXYZ", new HyperGraphElementCreator<EdgeSE3PointXYZ>);
      factory->registerType("EDGE_SE3_PRIOR", new HyperGraphElementCreator<EdgeSE3Prior>);
      factory->registerType("CACHE_SE3_OFFSET", new HyperGraphElementCreator<CacheSE3Offset>);
      factory->registerType("EDGE_SE3_OFFSET", new HyperGraphElementCreator<EdgeSE3Offset>);

      factory->registerType("PARAMS_CAMERACALIB", new HyperGraphElementCreator<ParameterCamera>);
      factory->registerType("CACHE_CAMERA", new HyperGraphElementCreator<CacheCamera>);
      factory->registerType("EDGE_PROJECT_DISPARITY", new HyperGraphElementCreator<EdgeSE3PointXYZDisparity>);
      factory->registerType("EDGE_PROJECT_DEPTH", new HyperGraphElementCreator<EdgeSE3PointXYZDepth>);


      HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
      actionLib->registerAction(new VertexSE3WriteGnuplotAction);
      actionLib->registerAction(new EdgeSE3WriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
      actionLib->registerAction(new VertexPointXYZDrawAction);
      actionLib->registerAction(new VertexSE3DrawAction);
      actionLib->registerAction(new EdgeSE3DrawAction);
      actionLib->registerAction(new CacheCameraDrawAction);
#endif
      types_slam3d::initialized = 1;
    }
  }

  G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_types)
  {
    types_slam3d::init();
  }

} // end namespace
