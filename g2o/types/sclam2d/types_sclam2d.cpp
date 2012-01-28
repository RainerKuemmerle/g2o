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

#include "vertex_odom_differential_params.h"

#include "edge_se2_sensor_calib.h"
#include "edge_se2_odom_differential_calib.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

namespace g2o {

  G2O_USE_TYPE_GROUP(slam2d);
  
  G2O_REGISTER_TYPE_GROUP(sclam);
      G2O_REGISTER_TYPE(VERTEX_ODOM_DIFFERENTIAL, VertexOdomDifferentialParams);
      G2O_REGISTER_TYPE(EDGE_SE2_CALIB, EdgeSE2SensorCalib);
      G2O_REGISTER_TYPE(EDGE_SE2_ODOM_DIFFERENTIAL_CALIB, EdgeSE2OdomDifferentialCalib);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(EdgeSE2SensorCalibDrawAction);
       G2O_REGISTER_ACTION(EdgeSE2OdomDifferentialCalibDrawAction);
#endif

} // end namespace
