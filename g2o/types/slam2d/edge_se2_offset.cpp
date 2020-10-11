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

#include "edge_se2_offset.h"

#include "parameter_se2_offset.h"

#include <iostream>

namespace g2o {
  using namespace std;

  EdgeSE2Offset::EdgeSE2Offset() : BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>() {
    information().setIdentity();
    _offsetFrom = 0;
    _offsetTo = 0;
    _cacheFrom = 0;
    _cacheTo = 0;
    resizeParameters(2);
    installParameter(_offsetFrom, 0);
    installParameter(_offsetTo, 1);
  }

  bool EdgeSE2Offset::resolveCaches(){
    assert(_offsetFrom && _offsetTo);

    ParameterVector pv(1);
    pv[0]=_offsetFrom;
    resolveCache(_cacheFrom, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE2_OFFSET",pv);
    pv[0]=_offsetTo;
    resolveCache(_cacheTo, (OptimizableGraph::Vertex*)_vertices[1],"CACHE_SE2_OFFSET",pv);
    return (_cacheFrom && _cacheTo);
  }

  bool EdgeSE2Offset::read(std::istream& is) {
    int pidFrom, pidTo;
    is >> pidFrom >> pidTo;
    if (! setParameterId(0,pidFrom))
      return false;
    if (! setParameterId(1,pidTo))
      return false;

    Vector3 meas;
    internal::readVector(is, meas);
    setMeasurement(SE2(meas));
    if (is.bad()) return false;
    readInformationMatrix(is);
    if (is.bad()) {
      //  we overwrite the information matrix with the Identity
      information().setIdentity();
    }
    return true;
  }

  bool EdgeSE2Offset::write(std::ostream& os) const {
    os << _offsetFrom->id() << " " << _offsetTo->id() << " ";
    internal::writeVector(os, measurement().toVector());
    return writeInformationMatrix(os);
  }

  void EdgeSE2Offset::computeError() {
    SE2 delta=_inverseMeasurement * _cacheFrom->w2n() * _cacheTo->n2w();
    _error.head<2>() = delta.translation();
    _error(2)=delta.rotation().angle();
  }

  bool EdgeSE2Offset::setMeasurementFromState(){
    SE2 delta = _cacheFrom->w2n() * _cacheTo->n2w();
    setMeasurement(delta);
    return true;
  }

  void EdgeSE2Offset::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE2 *from = static_cast<VertexSE2*>(_vertices[0]);
    VertexSE2 *to   = static_cast<VertexSE2*>(_vertices[1]);

    SE2 virtualMeasurement = _cacheFrom->offsetParam()->offset() * measurement() * _cacheTo->offsetParam()->offset().inverse();

    if (from_.count(from) > 0)
      to->setEstimate(from->estimate() * virtualMeasurement);
    else
      from->setEstimate(to->estimate() * virtualMeasurement.inverse());
  }

}
