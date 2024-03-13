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

#ifndef G2O_VERTEX_SE2_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE2_OFFSET_PARAMETERS_H_

#include <memory>

#include "g2o/core/cache.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/parameter.h"
#include "g2o_types_slam2d_api.h"
#include "se2.h"

namespace g2o {

class VertexSE2;

/**
 * \brief offset for an SE2
 */
class G2O_TYPES_SLAM2D_API ParameterSE2Offset : public BaseParameter<SE2> {
 public:
  ParameterSE2Offset();

  /**
   * update the offset to a new value.
   * re-calculates the different representations, e.g., the rotation matrix
   */
  void update() override;

  //! rotation of the offset as 2x2 rotation matrix
  [[nodiscard]] const Isometry2& offsetMatrix() const { return offsetMatrix_; }

  //! rotation of the inverse offset as 2x2 rotation matrix
  [[nodiscard]] const Isometry2& inverseOffsetMatrix() const {
    return inverseOffsetMatrix_;
  }

 protected:
  Isometry2 offsetMatrix_;
  Isometry2 inverseOffsetMatrix_;
};

/**
 * \brief caching the offset related to a vertex
 */
class G2O_TYPES_SLAM2D_API CacheSE2Offset : public Cache {
 public:
  using ParameterType = ParameterSE2Offset;

  CacheSE2Offset() = default;
  void updateImpl() override;

  [[nodiscard]] std::shared_ptr<ParameterSE2Offset> offsetParam() const {
    return std::static_pointer_cast<ParameterSE2Offset>(parameters_[0]);
  }

  [[nodiscard]] const SE2& w2n() const { return se2_w2n_; }
  [[nodiscard]] const SE2& n2w() const { return se2_n2w_; }

  [[nodiscard]] const Isometry2& w2nMatrix() const { return w2n_; }
  [[nodiscard]] const Isometry2& n2wMatrix() const { return n2w_; }
  [[nodiscard]] const Isometry2& w2lMatrix() const { return w2l_; }

  [[nodiscard]] Matrix2 RpInverseRInverseMatrix() const {
    return RpInverse_RInverse_;
  }
  [[nodiscard]] Matrix2 RpInverseRInversePrimeMatrix() const {
    return RpInverse_RInversePrime_;
  }

 protected:
  SE2 se2_w2n_;
  SE2 se2_n2w_;

  Isometry2 w2n_;  ///< world to sensor transform
  Isometry2 w2l_;  ///< world to local
  Isometry2 n2w_;  ///< sensor to world
  Matrix2 RpInverse_RInverse_;
  Matrix2 RpInverse_RInversePrime_;
};

}  // namespace g2o

#endif
