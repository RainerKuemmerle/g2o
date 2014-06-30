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

#include "edge_line3d.h"

namespace g2o
{

  EdgeLine3D::EdgeLine3D() :
    BaseBinaryEdge<6, Vector6d, VertexLine3D, VertexLine3D>()
  {
    _information.setIdentity();
    _error.setZero();
  }

  bool EdgeLine3D::read(std::istream& is)
  {
    Vector6d  v;
    for (int i = 0; i < 6; ++i)
      is >> v[i];
    setMeasurement(v);
    for (int i = 0; i < 6; ++i)
      for (int j = i; j < 6; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeLine3D::write(std::ostream& os) const
  {
    for (int i = 0; i < 6; ++i)
      os << _measurement[i] << " ";
    for (int i = 0; i < 6; ++i)
      for (int j = i; j < 6; ++j)
        os << " " << information()(i, j);
    return os.good();
  }


#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeLine3D::linearizeOplus()
  {
    _jacobianOplusXi=-Matrix6d::Identity();
    _jacobianOplusXj= Matrix6d::Identity();
  }
#endif


} // end namespace
