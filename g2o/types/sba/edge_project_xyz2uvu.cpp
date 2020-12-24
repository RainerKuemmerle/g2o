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

#include "edge_project_xyz2uvu.h"

namespace g2o {

EdgeProjectXYZ2UVU::EdgeProjectXYZ2UVU()
    : BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSE3Expmap>() {
  _cam = nullptr;
  resizeParameters(1);
  installParameter(_cam, 0);
}

void EdgeProjectXYZ2UVU::computeError() {
  const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[0]);
  const CameraParameters* cam = static_cast<const CameraParameters*>(parameter(0));
  _error = measurement() - cam->stereocam_uvu_map(v1->estimate().map(v2->estimate()));
}

bool EdgeProjectXYZ2UVU::read(std::istream& is) {
  readParamIds(is);
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeProjectXYZ2UVU::write(std::ostream& os) const {
  writeParamIds(os);
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

}  // namespace g2o
