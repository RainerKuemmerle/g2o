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

#include "g2o/config.h"

#include "robot_laser.h"

#include "g2o/core/factory.h"
#include "g2o/core/creators.h"
#include "g2o/stuff/macros.h"

namespace g2o {

  namespace types_data {
    int initialized = 0;
  }

  G2O_ATTRIBUTE_CONSTRUCTOR(init_types_data)
  {
    if (types_data::initialized)
      return;
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

    factory->registerType("ROBOTLASER1", new HyperGraphElementCreator<RobotLaser>);
    types_data::initialized = 1;
  }

} // end namespace
