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

#include "edge_plane.h"

namespace g2o
{
using namespace Eigen;

EdgePlane::EdgePlane() :
    BaseBinaryEdge<4, Vector4D, VertexPlane, VertexPlane>()
{
    _information.setIdentity();
    _error.setZero();
}

bool EdgePlane::read(std::istream& is)
{
  Vector4D  v;
    int size=4;
    for (int i = 0; i < size; ++i)
        is >> v[i];
    setMeasurement(v);
    for (int i = 0; i < size; ++i)
        for (int j = i; j < size; ++j) {
            is >> information()(i, j);
            if (i != j)
                information()(j, i) = information()(i, j);
        }
    return true;
}

bool EdgePlane::write(std::ostream& os) const
{
    int size=4;
    for (int i = 0; i < size; ++i)
        os << _measurement[i] << " ";
    for (int i = 0; i < size; ++i)
        for (int j = i; j < size; ++j)
            os << " " << information()(i, j);
    return os.good();
}

#if 0
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgePlane::linearizeOplus()
{
    _jacobianOplusXi=-Matrix3D::Identity();
    _jacobianOplusXj= Matrix3D::Identity();
}
#endif
#endif


} // end namespace
