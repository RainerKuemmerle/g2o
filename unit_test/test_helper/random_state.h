// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <Eigen/Geometry>
#include "g2o/config.h"
#include "g2o/core/eigen_types.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/se3quat.h"

namespace g2o {
namespace internal {
inline Isometry3 randomIsometry3() {
  const g2o::Vector3 rotAxisAngle = g2o::Vector3::Random() + g2o::Vector3::Random();
  const g2o::AngleAxis rotation(rotAxisAngle.norm(), rotAxisAngle.normalized());
  return g2o::Translation3(g2o::Vector3::Random()) * rotation;
}

struct RandomSE3Quat {
  static SE3Quat create() {
    SE3Quat result(Quaternion::UnitRandom(), Vector3::Random());
    result.normalizeRotation();
    return result;
  }
  static bool isApprox(const SE3Quat& a, const SE3Quat& b) {
    return a.toVector().isApprox(b.toVector(), 1e-5);
  }
};

struct RandomIsometry3 {
  static Isometry3 create() { return randomIsometry3(); }
  static bool isApprox(const Isometry3& a, const Isometry3& b) {
    return a.isApprox(b, 1e-5);
  }
};

struct RandomDouble {
  static double create() { return g2o::Sampler::uniformRand(-1., 1.); }
  static bool isApprox(const double& a, const double& b) {
    return std::abs(a - b) < 1e-5;
  }
};

}  // namespace internal
}  // namespace g2o
