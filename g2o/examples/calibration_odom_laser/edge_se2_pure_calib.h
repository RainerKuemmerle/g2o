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

#ifndef EDGE_SE2_PURE_CALIB_H
#define EDGE_SE2_PURE_CALIB_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/type_traits.h"
#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o_calibration_odom_laser_api.h"

namespace g2o {

struct G2O_CALIBRATION_ODOM_LASER_API OdomAndLaserMotion {
  VelocityMeasurement velocityMeasurement;
  SE2 laserMotion;
};

/**
 * @brief TypeTraits specialization for a SE2
 */
template <>
struct TypeTraits<OdomAndLaserMotion> {
  enum {
    kVectorDimension = 6,
    kMinimalVectorDimension = 6,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = OdomAndLaserMotion;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType res;
    res << TypeTraits<VelocityMeasurement>::toVector(t.velocityMeasurement),
        TypeTraits<SE2>::toVector(t.laserMotion);
    return res;
  }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = toVector(t);
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return toVector(t);
  }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = toVector(t);
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    OdomAndLaserMotion res;
    res.velocityMeasurement =
        TypeTraits<VelocityMeasurement>::fromVector(v.template head<3>());
    res.laserMotion = TypeTraits<SE2>::fromVector(v.template tail<3>());
    return res;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return fromVector(v);
  }

  static Type Identity() { return OdomAndLaserMotion(); }
};

/**
 * \brief calibrate odometry and laser based on a set of measurements
 */
class G2O_CALIBRATION_ODOM_LASER_API EdgeSE2PureCalib
    : public BaseBinaryEdge<3, OdomAndLaserMotion, VertexSE2,
                            VertexOdomDifferentialParams> {
 public:
  EdgeSE2PureCalib() = default;

  void computeError() override;
};

}  // namespace g2o

#endif
