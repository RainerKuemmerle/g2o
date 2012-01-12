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

#include "types_tutorial_slam2d.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  using namespace std;

  bool init_tutorial_slam2d_types()
  {
    cerr << __PRETTY_FUNCTION__ << " called" << endl;
    Factory* factory = Factory::instance();
    factory->registerType("TUTORIAL_VERTEX_SE2", new HyperGraphElementCreator<tutorial::VertexSE2>);
    factory->registerType("TUTORIAL_VERTEX_POINT_XY", new HyperGraphElementCreator<tutorial::VertexPointXY>);

    factory->registerType("TUTORIAL_PARAMS_SE2_OFFSET", new HyperGraphElementCreator<tutorial::ParameterSE2Offset>);

    factory->registerType("TUTORIAL_CACHE_SE2_OFFSET", new HyperGraphElementCreator<tutorial::CacheSE2Offset>);

    factory->registerType("TUTORIAL_EDGE_SE2", new HyperGraphElementCreator<tutorial::EdgeSE2>);
    factory->registerType("TUTORIAL_EDGE_SE2_POINT_XY", new HyperGraphElementCreator<tutorial::EdgeSE2PointXY>);
    return true;
  }

} // end namespace
