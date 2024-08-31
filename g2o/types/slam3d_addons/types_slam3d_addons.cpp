// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/config.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d_addons/edge_plane.h"
#include "g2o/types/slam3d_addons/edge_se3_calib.h"
#include "g2o/types/slam3d_addons/edge_se3_line.h"
#include "g2o/types/slam3d_addons/edge_se3_plane_calib.h"
#include "g2o/types/slam3d_addons/vertex_line3d.h"
#include "g2o/types/slam3d_addons/vertex_plane.h"

namespace g2o {

G2O_REGISTER_TYPE_GROUP(slam3d_addons);

G2O_REGISTER_TYPE_NAME("VERTEX_PLANE", VertexPlane);
G2O_REGISTER_TYPE_NAME("EDGE_SE3_PLANE_CALIB", EdgeSE3PlaneSensorCalib);

G2O_REGISTER_TYPE_NAME("VERTEX_LINE3D", VertexLine3D);
G2O_REGISTER_TYPE_NAME("EDGE_SE3_LINE3D", EdgeSE3Line3D);
G2O_REGISTER_TYPE_NAME("EDGE_PLANE", EdgePlane);
G2O_REGISTER_TYPE_NAME("EDGE_SE3_CALIB", EdgeSE3Calib);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(CacheCameraDrawAction);
G2O_REGISTER_ACTION(VertexPlaneDrawAction);
G2O_REGISTER_ACTION(EdgeSE3PlaneSensorCalibDrawAction);
G2O_REGISTER_ACTION(VertexLine3DDrawAction);
G2O_REGISTER_ACTION(EdgeSE3Line3DDrawAction);
#endif

}  // namespace g2o
