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

#include "vertex_se3_expmap.h"

#include "g2o/stuff/misc.h"

namespace g2o {

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7 est;
  internal::readVector(is, est);
  setEstimate(SE3Quat(est).inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  return internal::writeVector(os, estimate().inverse().toVector());
}

void VertexSE3Expmap::setToOriginImpl() { _estimate = SE3Quat(); }

void VertexSE3Expmap::oplusImpl(const number_t* update_) {
  Eigen::Map<const Vector6> update(update_);
  setEstimate(SE3Quat::exp(update) * estimate());
}

}  // namespace g2o
