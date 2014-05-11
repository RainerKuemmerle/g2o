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

#include "types_slam2d.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

  G2O_REGISTER_TYPE_GROUP(slam2d);

  G2O_REGISTER_TYPE(VERTEX_SE2, VertexSE2);
  G2O_REGISTER_TYPE(VERTEX_XY, VertexPointXY);
  G2O_REGISTER_TYPE(PARAMS_SE2OFFSET, ParameterSE2Offset);
  G2O_REGISTER_TYPE(CACHE_SE2_OFFSET, CacheSE2Offset);
  G2O_REGISTER_TYPE(EDGE_PRIOR_SE2, EdgeSE2Prior);
  G2O_REGISTER_TYPE(EDGE_PRIOR_SE2_XY, EdgeSE2XYPrior);
  G2O_REGISTER_TYPE(EDGE_SE2, EdgeSE2);
  G2O_REGISTER_TYPE(EDGE_SE2_XY, EdgeSE2PointXY);
  G2O_REGISTER_TYPE(EDGE_BEARING_SE2_XY, EdgeSE2PointXYBearing);
  G2O_REGISTER_TYPE(EDGE_SE2_XY_CALIB, EdgeSE2PointXYCalib);
  G2O_REGISTER_TYPE(EDGE_SE2_OFFSET, EdgeSE2Offset);
  G2O_REGISTER_TYPE(EDGE_SE2_POINTXY_OFFSET, EdgeSE2PointXYOffset);
  G2O_REGISTER_TYPE(EDGE_POINTXY, EdgePointXY);
  G2O_REGISTER_TYPE(EDGE_SE2_TWOPOINTSXY, EdgeSE2TwoPointsXY);
  G2O_REGISTER_TYPE(EDGE_SE2_LOTSOFXY, EdgeSE2LotsOfXY);

 
  G2O_REGISTER_ACTION(VertexSE2WriteGnuplotAction);
  G2O_REGISTER_ACTION(VertexPointXYWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2WriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYBearingWriteGnuplotAction);


#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(VertexSE2DrawAction);
  G2O_REGISTER_ACTION(VertexPointXYDrawAction);
  G2O_REGISTER_ACTION(EdgeSE2DrawAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYDrawAction);
  G2O_REGISTER_ACTION(EdgeSE2PointXYBearingDrawAction);

#endif
} // end namespace
