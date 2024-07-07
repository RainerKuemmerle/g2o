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

#ifndef G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_

#include <memory>

#include "g2o/config.h"
#include "g2o/core/cache.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/parameter.h"
#include "g2o/stuff/property.h"
#include "g2o_types_slam3d_api.h"
#include "type_traits_isometry3.h"  // IWYU pragma: keep

namespace g2o {

/**
 * \brief offset for an SE3
 */
class G2O_TYPES_SLAM3D_API ParameterSE3Offset
    : public BaseParameter<Isometry3> {
 public:
  ParameterSE3Offset();

  /**
   * update the offset to a new value.
   * re-calculates the different representations, e.g., the rotation matrix
   */
  void update() override;

  //! rotation of the inverse offset as 3x3 rotation matrix
  [[nodiscard]] const Isometry3& inverseOffset() const {
    return inverseOffset_;
  }

 protected:
  ParameterType inverseOffset_;
};

/**
 * \brief caching the offset related to a vertex
 */
class G2O_TYPES_SLAM3D_API CacheSE3Offset : public Cache {
 public:
  using ParameterType = ParameterSE3Offset;

  void updateImpl() override;

  [[nodiscard]] std::shared_ptr<ParameterType> offsetParam() const {
    return std::static_pointer_cast<ParameterType>(parameters_[0]);
  }
  void setOffsetParam(ParameterSE3Offset* offsetParam);

  [[nodiscard]] const Isometry3& w2n() const { return w2n_; }
  [[nodiscard]] const Isometry3& n2w() const { return n2w_; }
  [[nodiscard]] const Isometry3& w2l() const { return w2l_; }

 protected:
  Isometry3 w2n_;
  Isometry3 n2w_;
  Isometry3 w2l_;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM3D_API CacheSE3OffsetDrawAction : public DrawAction {
 public:
  CacheSE3OffsetDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  HyperGraphElementAction::Parameters& params_) override;

 protected:
  DrawAction::Parameters* refreshPropertyPtrs(
      HyperGraphElementAction::Parameters& params_) override;
  std::shared_ptr<FloatProperty> cubeSide_;
};
#endif

}  // namespace g2o

#endif
