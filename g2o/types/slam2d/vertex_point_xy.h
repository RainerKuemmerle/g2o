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

#ifndef G2O_VERTEX_POINT_XY_H
#define G2O_VERTEX_POINT_XY_H

#include <Eigen/Core>
#include <memory>

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/stuff/property.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API VertexPointXY : public BaseVertex<2, Vector2> {
 public:
  VertexPointXY();

  void oplusImpl(const VectorX::MapType& update) override {
    estimate_ += update.head<2>();
  }
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API VertexPointXYDrawAction : public DrawAction {
 public:
  VertexPointXYDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  HyperGraphElementAction::Parameters& params) override;

 protected:
  std::shared_ptr<FloatProperty> pointSize_;
  DrawAction::Parameters* refreshPropertyPtrs(
      HyperGraphElementAction::Parameters& params) override;
};
#endif

}  // namespace g2o

#endif
