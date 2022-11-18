// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_SBA_VERTEX_INTRINSICS_H
#define G2O_SBA_VERTEX_INTRINSICS_H

#include "g2o/core/base_vertex.h"
#include "g2o_types_sba_api.h"

namespace g2o {

/**
 * \brief Vertex encoding the intrinsics of the camera fx, fy, cx, xy, baseline;
 */
class G2O_TYPES_SBA_API VertexIntrinsics
    : public BaseVertex<4, Eigen::Matrix<number_t, 5, 1, Eigen::ColMajor> > {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexIntrinsics();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setToOriginImpl() override;

  void oplusImpl(const VectorX::MapType& update) override;
};

}  // namespace g2o

#endif
