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

#include "edge_line2d.h"

namespace g2o {

EdgeLine2D::EdgeLine2D() {
  information_.setIdentity();
  error_.setZero();
}

void EdgeLine2D::computeError() {
  const VertexLine2D* v1 = vertexXnRaw<0>();
  const VertexLine2D* v2 = vertexXnRaw<1>();
  for (int i = 0; i < 2; ++i)
    error_[i] = (v2->estimate()[i] - v1->estimate()[i]) - measurement_[i];
}

bool EdgeLine2D::setMeasurementFromState() {
  const VertexLine2D* v1 = vertexXnRaw<0>();
  const VertexLine2D* v2 = vertexXnRaw<1>();
  measurement_ = Line2D(TypeTraits<Line2D>::toVector(v2->estimate()) -
                        TypeTraits<Line2D>::toVector(v1->estimate()));
  return true;
}

void EdgeLine2D::linearizeOplus() {
  jacobianOplusXi_ = -Matrix2::Identity();
  jacobianOplusXj_ = Matrix2::Identity();
}

}  // namespace g2o
