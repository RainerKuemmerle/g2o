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

#ifndef G2O_EDGE_LINE2D_POINTXY_H
#define G2O_EDGE_LINE2D_POINTXY_H

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/stuff/misc.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o_types_slam2d_addons_api.h"
#include "vertex_line2d.h"

namespace g2o {

class EdgeLine2DPointXY
    : public BaseBinaryEdge<1, number_t, VertexLine2D,
                            VertexPointXY>  // Avoid redefinition of BaseEdge in
                                            // MSVC
{
 public:
  G2O_TYPES_SLAM2D_ADDONS_API EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      G2O_TYPES_SLAM2D_ADDONS_API void
      computeError() override {
    const VertexLine2D* l = vertexXnRaw<0>();
    const VertexPointXY* p = vertexXnRaw<1>();
    Vector2 n(std::cos(l->theta()), std::sin(l->theta()));
    number_t prediction = n.dot(p->estimate()) - l->rho();
    error_[0] = prediction - measurement_;
  }

  G2O_TYPES_SLAM2D_ADDONS_API bool setMeasurementData(
      const number_t* d) override {
    measurement_ = *d;
    return true;
  }

  G2O_TYPES_SLAM2D_ADDONS_API bool getMeasurementData(
      number_t* d) const override {
    *d = measurement_;
    return true;
  }

  G2O_TYPES_SLAM2D_ADDONS_API int measurementDimension() const override {
    return 1;
  }

  G2O_TYPES_SLAM2D_ADDONS_API bool setMeasurementFromState() override {
    const VertexLine2D* l = vertexXnRaw<0>();
    const VertexPointXY* p = vertexXnRaw<1>();
    Vector2 n(std::cos(l->theta()), std::sin(l->theta()));
    number_t prediction = n.dot(p->estimate()) - l->rho();
    measurement_ = prediction;
    return true;
  }

  G2O_TYPES_SLAM2D_ADDONS_API bool read(std::istream& is) override;
  G2O_TYPES_SLAM2D_ADDONS_API bool write(std::ostream& os) const override;
};

}  // namespace g2o

#endif
