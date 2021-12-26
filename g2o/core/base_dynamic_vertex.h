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

#ifndef G2O_BASE_DYNAMIC_VERTEX_H
#define G2O_BASE_DYNAMIC_VERTEX_H

#include "base_vertex.h"

namespace g2o {
template <typename T>
class BaseDynamicVertex : public BaseVertex<-1, T> {
 public:
  inline virtual bool setDimension(int newDimension);

 protected:
  // This method is responsible for actually changing the dimension of the state
  virtual bool setDimensionImpl(int newDimension) = 0;

  using BaseVertex<-1, T>::graph_;
  using BaseVertex<-1, T>::dimension_;
  using BaseVertex<-1, T>::b_;
  using BaseVertex<-1, T>::edges_;
  using BaseVertex<-1, T>::setHessianIndex;
  using BaseVertex<-1, T>::mapHessianMemory;
  using BaseVertex<-1, T>::updateCache;
};

template <typename T>
bool BaseDynamicVertex<T>::setDimension(int newDimension) {
  // Check the dimension is non-negative.
  assert(newDimension >= 0);
  if (newDimension < 0) return false;

  // Nothing to do if the dimension is unchanged.
  if (newDimension == dimension_) return true;

  // Change the state to the requested dimension
  if (!static_cast<bool>(setDimensionImpl(newDimension))) return false;

  // Store the old dimension and assign the new
  int oldDimension = dimension_;
  dimension_ = newDimension;

  // Reset the allocation associated with this vertex and update the cache
  setHessianIndex(-1);
  mapHessianMemory(nullptr);
  b_.resize(dimension_);
  updateCache();

  // If the dimension is being increased and this vertex is in a
  // graph, update the size of the Jacobian workspace just in case it
  // needs to grow.
  if ((newDimension > oldDimension) && (graph_ != nullptr)) {
    JacobianWorkspace& jacobianWorkspace = graph_->jacobianWorkspace();
    for (auto& e : edges_) {
      jacobianWorkspace.updateSize(e.lock().get());
    }
  }

  return true;
}

}  // namespace g2o
#endif
