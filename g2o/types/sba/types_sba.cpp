// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include "types_sba.h"

#include <memory>

#include "g2o/core/factory.h"
#include "g2o/types/sba/edge_project_p2mc.h"
#include "g2o/types/sba/edge_project_p2sc.h"
#include "g2o/types/sba/edge_sba_cam.h"
#include "g2o/types/sba/edge_sba_scale.h"
#include "g2o/types/sba/vertex_cam.h"
#include "g2o/types/sba/vertex_intrinsics.h"
#include "g2o/types/slam3d/se3quat.h"

namespace g2o {

G2O_REGISTER_TYPE_GROUP(sba);

G2O_REGISTER_TYPE(VERTEX_CAM, VertexCam);
G2O_REGISTER_TYPE(VERTEX_INTRINSICS, VertexIntrinsics);

G2O_REGISTER_TYPE(EDGE_PROJECT_P2MC, EdgeProjectP2MC);
G2O_REGISTER_TYPE(EDGE_PROJECT_P2SC, EdgeProjectP2SC);
G2O_REGISTER_TYPE(EDGE_CAM, EdgeSBACam);
G2O_REGISTER_TYPE(EDGE_SCALE, EdgeSBAScale);

}  // namespace g2o
