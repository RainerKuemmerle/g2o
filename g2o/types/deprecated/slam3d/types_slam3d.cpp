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

#include "types_slam3d.h"
#include "g2o/core/factory.h"

namespace g2o {
namespace deprecated {

  G2O_REGISTER_TYPE_GROUP(deprecated_slam3d);

  // cannot register the types right now, as the macros use the class name
  // which declares duplicate functions, if also linking with the default
  // slam3d types
#if 0
  G2O_REGISTER_TYPE(DEPRECATED_VERTEX_SE3:QUAT, VertexSE3);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3:QUAT, EdgeSE3);
  G2O_REGISTER_TYPE(DEPRECATED_VERTEX_TRACKXYZ, VertexPointXYZ);

  G2O_REGISTER_TYPE(DEPRECATED_PARAMS_SE3OFFSET, ParameterSE3Offset);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_TRACKXYZ, EdgeSE3PointXYZ);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_PRIOR, EdgeSE3Prior);
  G2O_REGISTER_TYPE(DEPRECATED_CACHE_SE3_OFFSET, CacheSE3Offset);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_OFFSET, EdgeSE3Offset);

  G2O_REGISTER_TYPE(DEPRECATED_PARAMS_CAMERACALIB, ParameterCamera);
  G2O_REGISTER_TYPE(DEPRECATED_CACHE_CAMERA, CacheCamera);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_PROJECT_DISPARITY, EdgeSE3PointXYZDisparity);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_PROJECT_DEPTH, EdgeSE3PointXYZDepth);

  /*********** ACTIONS ************/
  G2O_REGISTER_ACTION(VertexSE3WriteGnuplotAction);
  G2O_REGISTER_ACTION(VertexPointXYZWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE3WriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(VertexPointXYZDrawAction);
  G2O_REGISTER_ACTION(VertexSE3DrawAction);
  G2O_REGISTER_ACTION(EdgeSE3DrawAction);
  G2O_REGISTER_ACTION(CacheCameraDrawAction);
#endif
#endif

} // end namespace
} // end namespace
