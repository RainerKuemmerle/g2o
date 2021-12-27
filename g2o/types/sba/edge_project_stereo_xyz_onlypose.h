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

#ifndef G2O_SBA_EDGEPROJECTSTEREOXYZ_ONLYPOSE_H
#define G2O_SBA_EDGEPROJECTSTEREOXYZ_ONLYPOSE_H

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_sba_api.h"
#include "vertex_se3_expmap.h"

namespace g2o {

// Edge to optimize only the camera pose stereo
// TODO why does this exist? Fix point in optimization instead?
class G2O_TYPES_SBA_API EdgeStereoSE3ProjectXYZOnlyPose
    : public BaseUnaryEdge<3, Vector3, VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZOnlyPose() {}

  bool read(std::istream &is);

  bool write(std::ostream &os) const;

  void computeError();

  bool isDepthPositive();

  virtual void linearizeOplus();

  Vector3 cam_project(const Vector3 &trans_xyz) const;

  Vector3 Xw;
  number_t fx, fy, cx, cy, bf;
};

}  // namespace g2o

#endif
