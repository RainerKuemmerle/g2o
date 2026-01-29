// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Line vertex with 4D tangent space parameterization
// This vertex uses an oplus that matches the analytical Jacobian derivation

#ifndef G2O_VERTEX_LINE3D_TANGENT_H
#define G2O_VERTEX_LINE3D_TANGENT_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o_types_slam3d_addons_api.h"
#include "line3d.h"

namespace g2o {

/**
 * @brief Line vertex with 4D tangent space parameterization
 *
 * Increment parameters: ξ = (u₁, u₂, v₁, v₂)
 * - u₁, u₂: 2-DOF rotation of direction (in plane perpendicular to d)
 * - v₁, v₂: 2-DOF translation of moment (in plane perpendicular to d)
 *
 * This parameterization allows for fully analytical Jacobians in the edge.
 */
class G2O_TYPES_SLAM3D_ADDONS_API VertexLine3DTangent
    : public BaseVertex<4, Line3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexLine3DTangent() : BaseVertex<4, Line3D>(), color(1., 0.5, 0.) {}

  virtual void setToOriginImpl() override {
    // Default: line along X axis through origin
    _estimate = Line3D();
  }

  virtual void oplusImpl(const double* update) override {
    // Extract increments
    double u1 = update[0];
    double u2 = update[1];
    double v1 = update[2];
    double v2 = update[3];

    // Current estimate
    Vector3 d_old = _estimate.d();
    Vector3 w_old = _estimate.w();

    // Build orthonormal basis perpendicular to d
    Vector3 e1, e2;
    computeOrthonormalBasis(d_old, e1, e2);

    // Rotation vector (in plane perpendicular to d)
    Vector3 omega = u1 * e1 + u2 * e2;
    double theta = omega.norm();

    Matrix3 R_delta;
    if (theta < 1e-10) {
      // Small angle approximation: R ≈ I + [ω]×
      R_delta = Matrix3::Identity() + skew(omega);
    } else {
      // Rodrigues formula: R = I + sin(θ)[k]× + (1-cos(θ))[k]×²
      Vector3 axis = omega / theta;
      Matrix3 K = skew(axis);
      R_delta = Matrix3::Identity() + std::sin(theta) * K +
                (1.0 - std::cos(theta)) * K * K;
    }

    // Moment increment (in plane perpendicular to d)
    Vector3 delta_w = v1 * e1 + v2 * e2;

    // Synchronized update: rotate both d and w, then add moment increment
    Vector3 d_new = R_delta * d_old;
    Vector3 w_new = R_delta * w_old + delta_w;

    // Orthogonalization to maintain Plücker constraint d·w = 0
    w_new = w_new - d_new * (d_new.dot(w_new));

    // Normalize direction
    d_new.normalize();

    // Update estimate
    _estimate.setD(d_new);
    _estimate.setW(w_new);
  }

  virtual bool read(std::istream& is) override {
    Vector6 lv;
    for (int i = 0; i < 6; ++i) is >> lv[i];
    _estimate = Line3D(lv);
    return true;
  }

  virtual bool write(std::ostream& os) const override {
    for (int i = 0; i < 6; ++i) os << _estimate[i] << " ";
    return os.good();
  }

  virtual bool setEstimateDataImpl(const double* est) override {
    Eigen::Map<const Vector6> _est(est);
    _estimate = Line3D(_est);
    return true;
  }

  virtual bool getEstimateData(double* est) const override {
    Eigen::Map<Vector6> _est(est);
    _est = _estimate;
    return true;
  }

  virtual int estimateDimension() const override { return 6; }

  Vector3 color;

 private:
  static void computeOrthonormalBasis(const Vector3& d, Vector3& e1,
                                      Vector3& e2) {
    // Choose axis least aligned with d for numerical stability
    if (std::abs(d.x()) < std::abs(d.y()) &&
        std::abs(d.x()) < std::abs(d.z())) {
      e1 = d.cross(Vector3::UnitX()).normalized();
    } else if (std::abs(d.y()) < std::abs(d.z())) {
      e1 = d.cross(Vector3::UnitY()).normalized();
    } else {
      e1 = d.cross(Vector3::UnitZ()).normalized();
    }
    e2 = d.cross(e1);  // Already unit since d and e1 are orthonormal
  }

  static Matrix3 skew(const Vector3& v) {
    Matrix3 S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
  }
};

}  // namespace g2o

#endif
