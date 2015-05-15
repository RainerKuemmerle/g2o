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

#include "edge_se3_line.h"

namespace g2o {

  EdgeSE3Line3D::EdgeSE3Line3D(){
    information().setIdentity();
    cache = 0;
    offsetParam = 0;
    resizeParameters(1);
    installParameter(offsetParam, 0);
  }

  bool EdgeSE3Line3D::read(std::istream& is) {
    int pId;
    is >> pId;
    setParameterId(0, pId);

    for (int i=0; i<6; i++)
      is >> _measurement(i);
    _measurement(6) = 1.;
    information().setZero();
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
	is >> information()(i,j);
	information()(i,j) = information()(j,i);
      }
    information()(6,6) = 1e9; // normalization constraint
    return is.good();
  }

  bool EdgeSE3Line3D::write(std::ostream& os) const {
    os << offsetParam->id() << " ";
    for (int i=0; i<6; i++)
      os << _measurement(i) << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
	os << information()(i,j) << " ";
      }
    return os.good();
  }

  void EdgeSE3Line3D::computeError() {
    const VertexLine3D* landmark=static_cast<const VertexLine3D*>(vertices()[1]);
    Line3D projected(cache->w2n() * landmark->estimate());
    projected.normalize();
    _error.head(6) = _measurement.head<6>() - projected;
    _error(6) = 0; // this is the normalization constraint
  }

  bool EdgeSE3Line3D::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=offsetParam;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    return cache != 0;
  }

}
