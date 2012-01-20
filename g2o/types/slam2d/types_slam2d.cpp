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
    int initialized = 0;

    void init()
    {
      if (types_slam2d::initialized)
        return;

      Factory* factory = Factory::instance();
      //std::cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << std::endl;

      factory->registerType("VERTEX_SE2", new HyperGraphElementCreator<VertexSE2>);
      factory->registerType("VERTEX_XY", new HyperGraphElementCreator<VertexPointXY>);
      factory->registerType("PARAMS_SE2OFFSET", new HyperGraphElementCreator<ParameterSE2Offset>);
      factory->registerType("CACHE_SE2_OFFSET", new HyperGraphElementCreator<CacheSE2Offset>);

      factory->registerType("EDGE_PRIOR_SE2", new HyperGraphElementCreator<EdgeSE2Prior>);
      factory->registerType("EDGE_SE2", new HyperGraphElementCreator<EdgeSE2>);
      factory->registerType("EDGE_SE2_XY", new HyperGraphElementCreator<EdgeSE2PointXY>);
      factory->registerType("EDGE_BEARING_SE2_XY", new HyperGraphElementCreator<EdgeSE2PointXYBearing>);
      factory->registerType("EDGE_SE2_XY_CALIB", new HyperGraphElementCreator<EdgeSE2PointXYCalib>);
      factory->registerType("EDGE_SE2_OFFSET", new HyperGraphElementCreator<EdgeSE2Offset>);
      factory->registerType("EDGE_SE2_POINTXY_OFFSET", new HyperGraphElementCreator<EdgeSE2PointXYOffset>);

      HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();

      actionLib->registerAction(new VertexSE2WriteGnuplotAction);
      actionLib->registerAction(new VertexPointXYWriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2WriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2PointXYWriteGnuplotAction);
      actionLib->registerAction(new EdgeSE2PointXYBearingWriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
      actionLib->registerAction(new VertexSE2DrawAction);
      actionLib->registerAction(new VertexPointXYDrawAction);
      actionLib->registerAction(new EdgeSE2DrawAction);
      actionLib->registerAction(new EdgeSE2PointXYDrawAction);
      actionLib->registerAction(new EdgeSE2PointXYBearingDrawAction);
#endif

      types_slam2d::initialized = 1;

    }
  } // end namespace

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_slam2d)
  {
    types_slam2d::init();
  }

} // end namespace
