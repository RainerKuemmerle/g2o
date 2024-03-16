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

#include "g2o/core/factory.h"
#include "g2o/examples/tutorial_slam2d/edge_se2.h"
#include "g2o/examples/tutorial_slam2d/edge_se2_pointxy.h"
#include "g2o/examples/tutorial_slam2d/parameter_se2_offset.h"
#include "g2o/examples/tutorial_slam2d/vertex_point_xy.h"
#include "g2o/examples/tutorial_slam2d/vertex_se2.h"

namespace g2o::tutorial {

G2O_REGISTER_TYPE_GROUP(tutorial_slam2d);

G2O_REGISTER_TYPE(TUTORIAL_VERTEX_SE2, VertexSE2);
G2O_REGISTER_TYPE(TUTORIAL_VERTEX_POINT_XY, VertexPointXY);

G2O_REGISTER_TYPE(TUTORIAL_PARAMS_SE2_OFFSET, ParameterSE2Offset);

G2O_REGISTER_TYPE(TUTORIAL_CACHE_SE2_OFFSET, CacheSE2Offset);

G2O_REGISTER_TYPE(TUTORIAL_EDGE_SE2, EdgeSE2);
G2O_REGISTER_TYPE(TUTORIAL_EDGE_SE2_POINT_XY, EdgeSE2PointXY);
}  // namespace g2o::tutorial
