// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_SBA_CAMERAPARAMETERS_H
#define G2O_SBA_CAMERAPARAMETERS_H

#include <Eigen/Core>
#include <iosfwd>

#include "g2o/core/eigen_types.h"
#include "g2o/core/parameter.h"
#include "g2o/core/type_traits.h"
#include "g2o_types_sba_api.h"

namespace g2o {

struct StereoCameraParameters {
  double focal_length = 1.;
  Vector2 principle_point;
  double baseline = 0.5;
};

template <>
struct TypeTraits<StereoCameraParameters> {
  enum {
    kVectorDimension = 4,
    kMinimalVectorDimension = 4,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = StereoCameraParameters;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType result;
    result << t.focal_length, t.principle_point(0), t.principle_point(1),
        t.baseline;
    return result;
  }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = toVector(t);
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return toVector(t);
  }
  static void toMinimalData(const Type& t, double* data) { toData(t, data); }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    Type result;
    result.focal_length = v(0);
    result.principle_point(0) = v(1);
    result.principle_point(1) = v(2);
    result.baseline = v(3);
    return result;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return fromVector(v);
  }

  static Type Identity() { return Type(); }
};

class G2O_TYPES_SBA_API CameraParameters
    : public g2o::BaseParameter<StereoCameraParameters> {
 public:
  CameraParameters() = default;
  CameraParameters(double focalLength,
                   const Eigen::Ref<const Vector2>& principlePoint,
                   double baseLine);

  [[nodiscard]] Vector2 cam_map(const Vector3& trans_xyz) const;
  [[nodiscard]] Vector3 stereocam_uvu_map(const Vector3& trans_xyz) const;
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
};

}  // namespace g2o

#endif
