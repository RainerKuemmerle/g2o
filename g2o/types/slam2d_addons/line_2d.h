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

#ifndef G2O_LINE2D_H
#define G2O_LINE2D_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/types/slam2d/se2.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

struct Line2D : public Vector2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Line2D() { setZero(); }
  Line2D(const Vector2& v) {
    (*this)(0) = v(0);
    (*this)(1) = v(1);
  }
};

inline Line2D operator*(const SE2& t, const Line2D& l) {
  Line2D est = l;
  est[0] += t.rotation().angle();
  est[0] = normalize_theta(est[0]);
  Vector2 n(std::cos(est[0]), std::sin(est[0]));
  est[1] += n.dot(t.translation());
  return est;
}

}  // namespace g2o

#endif
