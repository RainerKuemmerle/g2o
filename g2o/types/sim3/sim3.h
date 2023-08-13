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
#include <cassert>
#include <utility>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/misc.h"
#include "g2o/types/slam3d/se3_ops.h"

namespace g2o {

struct Sim3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Quaternion r_;
  Vector3 t_;
  double s_;

 public:
  Sim3() {
    r_.setIdentity();
    t_.fill(0.);
    s_ = 1.;
  }

  Sim3(Quaternion r, Vector3 t, double s)
      : r_(std::move(r)), t_(std::move(t)), s_(s) {
    normalizeRotation();
  }

  Sim3(const Matrix3& R, Vector3 t, double s)
      : r_(Quaternion(R)), t_(std::move(t)), s_(s) {
    normalizeRotation();
  }

  explicit Sim3(const Vector7& update) {
    Vector3 omega;
    for (int i = 0; i < 3; i++) omega[i] = update[i];

    Vector3 upsilon;
    for (int i = 0; i < 3; i++) upsilon[i] = update[i + 3];

    double sigma = update[6];
    double theta = omega.norm();
    Matrix3 Omega = skew(omega);
    s_ = std::exp(sigma);
    Matrix3 Omega2 = Omega * Omega;
    Matrix3 I;
    I.setIdentity();
    Matrix3 R;

    constexpr double kEps = cst(0.00001);
    double A;
    double B;
    double C;
    if (fabs(sigma) < kEps) {
      C = 1;
      if (theta < kEps) {
        A = cst(1. / 2.);
        B = cst(1. / 6.);
        R = (I + Omega +
             Omega * Omega /
                 2);  // R=I+(1-cos(theta))*a^a^+sin(theta)*a^~=(omit
                      // O(theta^3))=I+theta^2/2*a^a^+theta*a^
      } else {
        double theta2 = theta * theta;
        A = (1 - std::cos(theta)) / (theta2);
        B = (theta - std::sin(theta)) / (theta2 * theta);
        R = I + std::sin(theta) / theta * Omega +
            (1 - std::cos(theta)) / (theta * theta) * Omega2;
      }
    } else {
      C = (s_ - 1) / sigma;
      if (theta < kEps) {
        double sigma2 = sigma * sigma;
        A = ((sigma - 1) * s_ + 1) / sigma2;
        B = ((cst(0.5) * sigma2 - sigma + 1) * s_ - 1) /
            (sigma2 *
             sigma);  // B=[C-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2~=(omit
                      // O(theta^2))=
        //(1/2*s*sigma-s)/(sigma^2)+[C-(s-1)*sigma/(sigma^2+theta^2)]/theta^2~=(0.5*sigma^2*s-s*sigma)/sigma^3+[s-1]/sigma^3=[s*(0.5*sigma^2-sigma+1)-1]/sigma^3
        R = (I + Omega +
             Omega2 /
                 2);  // R=I+(1-cos(theta))*a^a^+sin(theta)*a^~=I+theta^2/2*a^a^+theta*a^
      } else {
        R = I + std::sin(theta) / theta * Omega +
            (1 - std::cos(theta)) / (theta * theta) * Omega2;
        double a = s_ * std::sin(theta);
        double b = s_ * std::cos(theta);
        double theta2 = theta * theta;
        double sigma2 = sigma * sigma;
        double c = theta2 + sigma2;
        A = (a * sigma + (1 - b) * theta) / (theta * c);
        B = (C - ((b - 1) * sigma + a * theta) / (c)) * 1 / (theta2);
      }
    }
    r_ = Quaternion(R);

    Matrix3 W = A * Omega + B * Omega2 + C * I;
    t_ = W * upsilon;
  }

  [[nodiscard]] Vector3 map(const Vector3& xyz) const {
    return s_ * (r_ * xyz) + t_;
  }

  [[nodiscard]] Vector7 log() const {
    Vector7 res;
    double sigma = std::log(s_);

    Vector3 omega;
    Vector3 upsilon;

    Matrix3 R = r_.toRotationMatrix();
    double d = cst(0.5) * (R(0, 0) + R(1, 1) + R(2, 2) - 1);

    Matrix3 Omega;

    constexpr double kEps = cst(0.00001);
    Matrix3 I = Matrix3::Identity();

    double A;
    double B;
    double C;
    if (fabs(sigma) < kEps) {
      C = 1;
      if (d > 1 - kEps) {
        omega = 0.5 * deltaR(R);
        Omega = skew(omega);
        A = cst(1. / 2.);
        B = cst(1. / 6.);
      } else {
        double theta = std::acos(d);
        double theta2 = theta * theta;
        omega = theta / (2 * std::sqrt(1 - d * d)) * deltaR(R);
        Omega = skew(omega);
        A = (1 - std::cos(theta)) / (theta2);
        B = (theta - std::sin(theta)) / (theta2 * theta);
      }
    } else {
      C = (s_ - 1) / sigma;
      if (d > 1 - kEps) {
        double sigma2 = sigma * sigma;
        omega = cst(0.5) * deltaR(R);
        Omega = skew(omega);
        A = ((sigma - 1) * s_ + 1) / (sigma2);
        B = ((cst(0.5) * sigma2 - sigma + 1) * s_ - 1) /
            (sigma2 *
             sigma);  // B=[C-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2
        // use
        // limit(theta->0)(B)=limit(theta->0){[(sigma2+theta2)*(s*sigma*sin(theta)-s*sin(theta)-s*theta*cos(theta))+(s*cos(theta)*sigma-sigma+s*sin(theta)*theta)*2*theta]/(2*theta)}=
        //=limit(theta->0)(s*sigma-s)*sin(theta)/(2*(sigma2+theta2)*theta)+limit(theta->0)[-s*cos(theta)/(2*(sigma2+theta2))+(s*cos(theta)*sigma-sigma+s*sin(theta)*theta)/(sigma2+theta2)^2]=
        //=limit(theta->0)(s*sigma-s)*cos(theta)/(2*(sigma2+3*theta2))+-s/(2*sigma2)+(s-1)/sigma^3=
        //=(s*sigma-s)/2/sigma2-s/2/sigma2+(s-1)/sigma^3=[(0.5*sigma2-sigma+1)*s-1]/sigma^3
      } else {
        double theta = std::acos(d);
        omega = theta / (2 * std::sqrt(1 - d * d)) * deltaR(R);
        Omega = skew(omega);
        double theta2 = theta * theta;
        double a = s_ * std::sin(theta);
        double b = s_ * std::cos(theta);
        double c = theta2 + sigma * sigma;
        A = (a * sigma + (1 - b) * theta) / (theta * c);
        B = (C - ((b - 1) * sigma + a * theta) / (c)) * 1 / (theta2);
      }
    }

    Matrix3 W = A * Omega + B * Omega * Omega + C * I;

    upsilon = W.lu().solve(t_);

    for (int i = 0; i < 3; i++) res[i] = omega[i];

    for (int i = 0; i < 3; i++) res[i + 3] = upsilon[i];

    res[6] = sigma;

    return res;
  }

  [[nodiscard]] Sim3 inverse() const {
    return Sim3(r_.conjugate(), r_.conjugate() * ((-1 / s_) * t_), 1 / s_);
  }

  double operator[](int i) const {
    assert(i < 8);
    if (i < 4) {
      return r_.coeffs()[i];
    }
    if (i < 7) {
      return t_[i - 4];
    }
    return s_;
  }

  double& operator[](int i) {
    assert(i < 8);
    if (i < 4) {
      return r_.coeffs()[i];
    }
    if (i < 7) {
      return t_[i - 4];
    }
    return s_;
  }

  Sim3 operator*(const Sim3& other) const {
    Sim3 ret;
    ret.r_ = r_ * other.r_;
    ret.t_ = s_ * (r_ * other.t_) + t_;
    ret.s_ = s_ * other.s_;
    return ret;
  }

  Sim3& operator*=(const Sim3& other) {
    Sim3 ret = (*this) * other;
    *this = ret;
    return *this;
  }
  void normalizeRotation() {
    if (r_.w() < 0) {
      r_.coeffs() *= -1;
    }
    r_.normalize();
  }
  [[nodiscard]] const Vector3& translation() const { return t_; }

  Vector3& translation() { return t_; }

  [[nodiscard]] const Quaternion& rotation() const { return r_; }

  Quaternion& rotation() { return r_; }

  [[nodiscard]] const double& scale() const { return s_; }

  double& scale() { return s_; }
};

inline std::ostream& operator<<(std::ostream& out_str, const Sim3& sim3) {
  out_str << sim3.rotation().coeffs() << std::endl;
  out_str << sim3.translation() << std::endl;
  out_str << sim3.scale() << std::endl;

  return out_str;
}

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
