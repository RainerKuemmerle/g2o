// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#include "types_six_dof_expmap.h"

#include "g2o/core/factory.h"

namespace g2o {

// clang-format off
G2O_REGISTER_TYPE_GROUP(expmap);
G2O_REGISTER_TYPE(VERTEX_SE3:EXPMAP, VertexSE3Expmap);
G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP, EdgeSE3Expmap);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UV:EXPMAP, EdgeProjectXYZ2UV);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UVU:EXPMAP, EdgeProjectXYZ2UVU);
G2O_REGISTER_TYPE(EDGE_SE3_PROJECT_XYZ:EXPMAP, EdgeSE3ProjectXYZ);
G2O_REGISTER_TYPE(EDGE_SE3_PROJECT_XYZONLYPOSE:EXPMAP, EdgeSE3ProjectXYZOnlyPose);
G2O_REGISTER_TYPE(EDGE_STEREO_SE3_PROJECT_XYZ:EXPMAP, EdgeStereoSE3ProjectXYZ);
G2O_REGISTER_TYPE(EDGE_STEREO_SE3_PROJECT_XYZONLYPOSE:EXPMAP, EdgeStereoSE3ProjectXYZOnlyPose);
G2O_REGISTER_TYPE(PARAMS_CAMERAPARAMETERS, CameraParameters);
// clang-format on

}  // namespace g2o
