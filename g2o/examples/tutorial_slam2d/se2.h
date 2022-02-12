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

#ifndef G2O_TUTORIAL_SE2_H
#define G2O_TUTORIAL_SE2_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

namespace tutorial {

class G2O_TUTORIAL_SLAM2D_API SE2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE2() : R_(0), t_(0, 0) {}

  SE2(double x, double y, double theta) : R_(theta), t_(x, y) {}

  const Eigen::Vector2d& translation() const { return t_; }

  Eigen::Vector2d& translation() { return t_; }

  const Eigen::Rotation2Dd& rotation() const { return R_; }

  Eigen::Rotation2Dd& rotation() { return R_; }

  SE2 operator*(const SE2& tr2) const {
    SE2 result(*this);
    result.t_ += R_ * tr2.t_;
    result.R_.angle() += tr2.R_.angle();
    result.R_.angle() = normalize_theta(result.R_.angle());
    return result;
  }

  SE2& operator*=(const SE2& tr2) {
    t_ += R_ * tr2.t_;
    R_.angle() += tr2.R_.angle();
    R_.angle() = normalize_theta(R_.angle());
    return *this;
  }

  Eigen::Vector2d operator*(const Eigen::Vector2d& v) const {
    return t_ + R_ * v;
  }

  SE2 inverse() const {
    SE2 ret;
    ret.R_ = R_.inverse();
    ret.R_.angle() = normalize_theta(ret.R_.angle());
    ret.t_ = ret.R_ * (Eigen::Vector2d(-1 * t_));
    return ret;
  }

  double operator[](int i) const {
    assert(i >= 0 && i < 3);
    if (i < 2) return t_(i);
    return R_.angle();
  }

  double& operator[](int i) {
    assert(i >= 0 && i < 3);
    if (i < 2) return t_(i);
    return R_.angle();
  }

  void fromVector(const Eigen::Vector3d& v) { *this = SE2(v[0], v[1], v[2]); }

  Eigen::Vector3d toVector() const {
    Eigen::Vector3d ret;
    for (int i = 0; i < 3; i++) {
      ret(i) = (*this)[i];
    }
    return ret;
  }

 protected:
  Eigen::Rotation2Dd R_;
  Eigen::Vector2d t_;
};

}  // namespace tutorial
}  // namespace g2o

#endif
