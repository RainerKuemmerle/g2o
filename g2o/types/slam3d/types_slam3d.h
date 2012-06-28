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

#ifndef G2O_TYPES_SLAM3D_
#define G2O_TYPES_SLAM3D_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"

#define THREE_D_TYPES_ANALYTIC_JACOBIAN

#include "vertex_se3.h"
#include "edge_se3.h"
#include "vertex_pointxyz.h"

#include "parameter_se3_offset.h"
#include "edge_se3_pointxyz.h"
#include "edge_se3_offset.h"

#include "parameter_camera.h"
#include "parameter_stereo_camera.h"
#include "edge_se3_pointxyz_disparity.h"
#include "edge_se3_pointxyz_depth.h"
#include "edge_se3_prior.h"


#endif
