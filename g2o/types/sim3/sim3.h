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

#ifndef G2O_SIM_3
#define G2O_SIM_3

#include <Eigen/Geometry>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"

namespace g2o {

struct Sim3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Quaternion r_;
  Vector3 t_;
  double s_;

 public:
  Sim3();

  Sim3(Quaternion r, Vector3 t, double s);

  Sim3(const Matrix3& R, Vector3 t, double s);

  explicit Sim3(const Vector7& update);

  [[nodiscard]] Vector3 map(const Vector3& xyz) const {
    return s_ * (r_ * xyz) + t_;
  }

  [[nodiscard]] Vector7 log() const;

  [[nodiscard]] Sim3 inverse() const;

  double operator[](int i) const;

  double& operator[](int i);

  Sim3 operator*(const Sim3& other) const;
  Sim3& operator*=(const Sim3& other);

  void normalizeRotation();

  [[nodiscard]] const Vector3& translation() const { return t_; }

  Vector3& translation() { return t_; }

  [[nodiscard]] const Quaternion& rotation() const { return r_; }

  Quaternion& rotation() { return r_; }

  [[nodiscard]] const double& scale() const { return s_; }

  double& scale() { return s_; }
};

std::ostream& operator<<(std::ostream& out_str, const Sim3& sim3);

/**
 * @brief TypeTraits specialization for a Sim3
 */
template <>
struct TypeTraits<Sim3> {
  enum {
    kVectorDimension = 8,
    kMinimalVectorDimension = 7,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = Sim3;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorN<8> v;
    for (int i = 0; i < 8; ++i) v[i] = t[i];
    return v;
  }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    for (int i = 0; i < 8; ++i) v[i] = t[i];
  }

  static MinimalVectorType toMinimalVector(const Type& t) { return t.log(); }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.log();
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    Sim3 aux;
    for (int i = 0; i < 8; ++i) aux[i] = v[i];
    aux.normalizeRotation();
    return aux;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return Sim3(v);
  }

  static Type Identity() { return Sim3(); }
};

}  // namespace g2o

#endif
