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

#include "edge_se2_pointxy_offset.h"

#include <iostream>

#include "parameter_se2_offset.h"

namespace g2o {

// point to camera projection, monocular
EdgeSE2PointXYOffset::EdgeSE2PointXYOffset() {
  information().setIdentity();
  resizeParameters(1);
  installParameter<CacheSE2Offset::ParameterType>(0);
}

bool EdgeSE2PointXYOffset::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheSE2Offset>(vertexXn<0>(), "CACHE_SE2_OFFSET", pv);
  return cache_ != nullptr;
}

bool EdgeSE2PointXYOffset::read(std::istream &is) {
  int pId;
  is >> pId;
  setParameterId(0, pId);
  // measured keypoint
  internal::readVector(is, measurement_);
  if (is.bad()) return false;
  readInformationMatrix(is);
  //  we overwrite the information matrix in case of read errors
  if (is.bad()) information().setIdentity();
  return true;
}

bool EdgeSE2PointXYOffset::write(std::ostream &os) const {
  os << parameters_[0]->id() << " ";
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2PointXYOffset::computeError() {
  // from cam to point (track)
  // VertexSE2 *rob = static_cast<VertexSE2*>(vertices_[0]);
  VertexPointXY *point = vertexXnRaw<1>();

  Vector2 perr = cache_->w2lMatrix() * point->estimate();

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = perr - measurement_;
}

void EdgeSE2PointXYOffset::linearizeOplus() {
  VertexSE2 *rob = vertexXnRaw<0>();
  VertexPointXY *point = vertexXnRaw<1>();
  jacobianOplusXi_.block<2, 2>(0, 0) = -cache_->RpInverseRInverseMatrix();
  jacobianOplusXi_.block<2, 1>(0, 2) =
      cache_->RpInverseRInversePrimeMatrix() *
      (point->estimate() - rob->estimate().translation());
  jacobianOplusXj_ = cache_->RpInverseRInverseMatrix();
}

bool EdgeSE2PointXYOffset::setMeasurementFromState() {
  VertexPointXY *point = vertexXnRaw<1>();

  measurement_ = cache_->w2lMatrix() * point->estimate();
  return true;
}

void EdgeSE2PointXYOffset::initialEstimate(
    const OptimizableGraph::VertexSet &from,
    OptimizableGraph::Vertex * /*to_*/) {
  (void)from;
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexDepthCam position by VertexTrackXY");

  VertexSE2 *cam = vertexXnRaw<0>();
  VertexPointXY *point = vertexXnRaw<1>();
  // SE2OffsetCache* vcache = (SE2OffsetCache* ) cam->getCache(_cacheIds[0]);
  // if (! vcache){
  //   cerr << "fatal error in retrieving cache" << endl;
  // }
  // SE2OffsetParameters* params=vcache->params;
  Vector2 p = measurement_;
  point->setEstimate(cam->estimate() *
                     (cache_->offsetParam()->offsetMatrix() * p));
}

}  // namespace g2o
