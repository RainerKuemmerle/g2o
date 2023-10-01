// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_TYPES_ICP_EDGE_GICP_H
#define G2O_TYPES_ICP_EDGE_GICP_H

#include <Eigen/Core>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o_types_icp_api.h"

namespace g2o {
template <typename T>
struct TypeTraits;

/**
 * @brief class for edges between two points rigidly attached to vertices
 *
 */
class G2O_TYPES_ICP_API EdgeGICP {
 public:
  // point positions
  Vector3 pos0, pos1;

  // unit normals
  Vector3 normal0, normal1;

  // rotation matrix for normal
  Matrix3 R0, R1;

  // initialize an object
  EdgeGICP();

  // set up rotation matrix for pos0
  void makeRot0();

  // set up rotation matrix for pos1
  void makeRot1();

  // returns a precision matrix for point-plane
  Matrix3 prec0(double e);

  // returns a precision matrix for point-plane
  Matrix3 prec1(double e);

  // return a covariance matrix for plane-plane
  Matrix3 cov0(double e);

  // return a covariance matrix for plane-plane
  Matrix3 cov1(double e);
};

/**
 * @brief TypeTraits specialization for a SE2
 */
template <>
struct TypeTraits<EdgeGICP> {
  enum {
    kVectorDimension = 12,
    kMinimalVectorDimension = 12,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = EdgeGICP;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType res;
    int idx = 0;
    // first point
    for (int i = 0; i < 3; i++) res[idx++] = t.pos0[i];
    for (int i = 0; i < 3; i++) res[idx++] = t.normal0[i];

    // second point
    for (int i = 0; i < 3; i++) res[idx++] = t.pos1[i];
    for (int i = 0; i < 3; i++) res[idx++] = t.normal1[i];
    return res;
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
    Type t;
    int idx = 0;
    // first point
    for (int i = 0; i < 3; i++) t.pos0[i] = v[idx++];
    for (int i = 0; i < 3; i++) t.normal0[i] = v[idx++];

    // second point
    for (int i = 0; i < 3; i++) t.pos1[i] = v[idx++];
    for (int i = 0; i < 3; i++) t.normal1[i] = v[idx++];

    t.makeRot0();
    t.makeRot1();

    return t;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return fromVector(v);
  }

  static Type Identity() { return Type(); }
};

}  // namespace g2o

#endif  // TYPES_ICP
