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

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  using namespace std;

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_interactive_online)
  {
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

    factory->registerType("ONLINE_EDGE_SE2", new HyperGraphElementCreator<OnlineEdgeSE2>);
    factory->registerType("ONLINE_VERTEX_SE2", new HyperGraphElementCreator<OnlineVertexSE2>);

    factory->registerType("ONLINE_VERTEX_SE3:QUAT", new HyperGraphElementCreator<OnlineVertexSE3>);
    factory->registerType("ONLINE_EDGE_SE3:QUAT", new HyperGraphElementCreator<OnlineEdgeSE3>);
  }

} // end namespace
