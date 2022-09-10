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

#include "vertex_se2.h"

namespace g2o {
namespace tutorial {

void ParameterSE2Offset::setOffset(const SE2& offset) {
  offset_ = offset;
  inverseOffset_ = offset.inverse();
}

bool ParameterSE2Offset::read(std::istream& is) {
  double x;
  double y;
  double th;
  is >> x >> y >> th;
  setOffset(SE2(x, y, th));
  return true;
}

bool ParameterSE2Offset::write(std::ostream& os) const {
  os << offset_.translation().x() << " " << offset_.translation().y() << " "
     << offset_.rotation().angle();
  return os.good();
}

void CacheSE2Offset::updateImpl() {
#ifndef NDEBUG
  auto* offsetParam =
      dynamic_cast<ParameterSE2Offset*>(parameters_[0].get());
#else
  auto* offsetParam = static_cast<ParameterSE2Offset*>(parameters_[0].get());
#endif
  const auto& v = static_cast<const VertexSE2&>(vertex());
  n2w_ = v.estimate() * offsetParam->offset();
  w2n_ = n2w_.inverse();
}

}  // end namespace tutorial
}  // namespace g2o
