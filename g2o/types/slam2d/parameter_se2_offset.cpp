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

#include "parameter_se2_offset.h"

#include "g2o/core/io_helper.h"
#include "vertex_se2.h"

namespace g2o {

ParameterSE2Offset::ParameterSE2Offset() { setOffset(); }

void ParameterSE2Offset::setOffset(const SE2& offset) {
  offset_ = offset;
  offsetMatrix_ = offset_.rotation().toRotationMatrix();
  offsetMatrix_.translation() = offset_.translation();
  inverseOffsetMatrix_ = offsetMatrix_.inverse();
}

bool ParameterSE2Offset::read(std::istream& is) {
  Vector3 off;
  bool state = g2o::internal::readVector(is, off);
  setOffset(SE2(off));
  return state;
}

bool ParameterSE2Offset::write(std::ostream& os) const {
  return internal::writeVector(os, offset().toVector());
}

void CacheSE2Offset::updateImpl() {
#ifndef NDEBUG
  auto* offsetParam = dynamic_cast<ParameterSE2Offset*>(parameters_[0].get());
#else
  auto* offsetParam = static_cast<ParameterSE2Offset*>(parameters_[0].get());
#endif

  const auto& v = static_cast<const VertexSE2&>(vertex());
  se2_n2w_ = v.estimate() * offsetParam->offset();

  n2w_ = se2_n2w_.rotation().toRotationMatrix();
  n2w_.translation() = se2_n2w_.translation();

  se2_w2n_ = se2_n2w_.inverse();
  w2n_ = se2_w2n_.rotation().toRotationMatrix();
  w2n_.translation() = se2_w2n_.translation();

  SE2 w2l = v.estimate().inverse();
  w2l_ = w2l.rotation().toRotationMatrix();
  w2l_.translation() = w2l.translation();

  number_t alpha = v.estimate().rotation().angle();
  number_t c = std::cos(alpha);
  number_t s = std::sin(alpha);
  Matrix2 RInversePrime;
  RInversePrime << -s, c, -c, -s;
  RpInverse_RInversePrime_ =
      offsetParam->offset().rotation().toRotationMatrix().transpose() *
      RInversePrime;
  RpInverse_RInverse_ = w2l.rotation();
}

}  // namespace g2o
