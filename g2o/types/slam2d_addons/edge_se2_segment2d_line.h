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

#ifndef G2O_EDGE_SE2_SEGMENT2D_LINE_H
#define G2O_EDGE_SE2_SEGMENT2D_LINE_H

#include <Eigen/Core>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o_types_slam2d_addons_api.h"
#include "vertex_segment2d.h"

namespace g2o {

class EdgeSE2Segment2DLine
    : public BaseBinaryEdge<2, Vector2, VertexSE2,
                            VertexSegment2D>  // Avoid redefinition of BaseEdge
                                              // in MSVC
{
 public:
  G2O_TYPES_SLAM2D_ADDONS_API [[nodiscard]] double theta() const {
    return measurement_[0];
  }
  G2O_TYPES_SLAM2D_ADDONS_API [[nodiscard]] double rho() const {
    return measurement_[1];
  }

  G2O_TYPES_SLAM2D_ADDONS_API void setTheta(double t) { measurement_[0] = t; }
  G2O_TYPES_SLAM2D_ADDONS_API void setRho(double r) { measurement_[1] = r; }

  G2O_TYPES_SLAM2D_ADDONS_API void computeError() override;

  G2O_TYPES_SLAM2D_ADDONS_API bool setMeasurementFromState() override;

  /* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
  /*       virtual void linearizeOplus(); */
  /* #endif */
};

}  // namespace g2o

#endif
