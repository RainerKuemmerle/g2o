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

#include "edge_pointxy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  EdgePointXY::EdgePointXY() :
    BaseBinaryEdge<2, Vector2D, VertexPointXY, VertexPointXY>()
  {
    _information.setIdentity();
    _error.setZero();
  }

  bool EdgePointXY::read(std::istream& is)
  {
    Vector2D p;
    is >> p[0] >> p[1];
    setMeasurement(p);
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgePointXY::write(std::ostream& os) const
  {
    Vector2D p = measurement();
    os << p.x() << " " << p.y();
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j)
        os << " " << information()(i, j);
    return os.good();
  }


#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgePointXY::linearizeOplus()
  {
    _jacobianOplusXi=-Matrix2D::Identity();
    _jacobianOplusXj= Matrix2D::Identity();
  }
#endif


} // end namespace
