// g2o - General Graph Optimization
// Copyright (C) 2023 R. Kuemmerle
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

#ifndef G2O_TYPE_TRAITS_H
#define G2O_TYPE_TRAITS_H

#include <climits>

#include "eigen_types.h"

namespace g2o {

/**
 * @brief TypeTrait methods for types to be used inside g2o.
 *
 * @tparam T The type of an estimate or measurement
 * @tparam VectorDimension The dimension as a vector
 * @tparam IsVector whether is a vector type
 */
template <typename T>
struct TypeTraits {
  // enum {
  //   kVectorDimension = INT_MIN,  ///< dimension of the type as vector
  //   kMinimalVectorDimension =
  //       INT_MIN,    ///< dimension of the type as minimal vector
  //   kIsVector = 0,  ///< type is a vector
  //   kIsScalar = 0,  ///< type is a scalar value
  // };

  // using Type = T;
  // using VectorType = VectorN<kVectorDimension>;
  // using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  // static VectorType toVector(const Type& t);
  // static void toData(const Type& t, double* data);

  // static VectorType toMinimalVector(const Type& t);
  // static void toMinimalData(const Type& t, double* data);

  // template <typename Derived>
  // static Type fromVector(const Eigen::DenseBase<Derived>& v);

  // template <typename Derived>
  // static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v);

  // static Type Identity();
};

/**
 * @brief TypeTrait specialization for a vector type.
 *
 * @tparam N The dimension of the vector
 */
template <int N>
struct TypeTraits<VectorN<N>> {
  enum {
    kVectorDimension = N,
    kMinimalVectorDimension = N,
    kIsVector = 1,
    kIsScalar = 0,
  };
  using Type = VectorN<N>;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t; }
  static void toData(const Type& t, double* data) {  // NOLINT
    typename VectorType::MapType v(data, kVectorDimension);
    v = t;
  }

  static MinimalVectorType toMinimalVector(const Type& t) { return t; }
  static void toMinimalData(const Type& t, double* data) {  // NOLINT
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t;
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return v;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return v;
  }

  static Type Identity() { return Type::Zero(kVectorDimension); }
};

// explicitly instantiate
template struct TypeTraits<VectorN<1>>;
template struct TypeTraits<VectorN<2>>;
template struct TypeTraits<VectorN<3>>;
template struct TypeTraits<VectorN<4>>;
template struct TypeTraits<VectorN<5>>;
template struct TypeTraits<VectorN<6>>;
template struct TypeTraits<VectorN<7>>;

/**
 * @brief TypeTrait specialization for a scalar type.
 *
 * @tparam N The dimension of the vector
 */
template <>
struct TypeTraits<double> {
  enum {
    kVectorDimension = 1,
    kMinimalVectorDimension = 1,
    kIsVector = 0,
    kIsScalar = 1,
  };
  using Type = double;
  using VectorType = VectorN<1>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType res;
    res(0) = t;
    return res;
  }
  static void toData(const Type& t, double* data) {  // NOLINT
    data[0] = t;
  }

  static VectorType toMinimalVector(const Type& t) {
    VectorType res;
    res(0) = t;
    return res;
  }
  static void toMinimalData(const Type& t, double* data) {  // NOLINT
    data[0] = t;
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return v[0];
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return v[0];
  }

  static Type Identity() { return 0.; }
};

/**
 * @brief A helper trait to retrieve the dimension of a type.
 *
 * @tparam T The type for which to obtain the dimension
 */
template <typename T>
struct DimensionTraits {
  //! for a statically known type
  template <int IsVector = TypeTraits<T>::kIsVector,
            int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsVector == 0 && IsScalar == 0, int>::type
  dimension(const T&) {
    return TypeTraits<T>::kVectorDimension;
  }

  //! for a vector type
  template <int IsVector = TypeTraits<T>::kIsVector,
            int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsVector != 0 && IsScalar == 0, int>::type
  dimension(const T& t) {
    return t.size();
  }

  //! for a scalar value
  template <int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsScalar != 0, int>::type dimension(const T&) {
    return 1;
  }

  //! for a statically known type
  template <int IsVector = TypeTraits<T>::kIsVector,
            int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsVector == 0 && IsScalar == 0, int>::type
  minimalDimension(const T&) {
    return TypeTraits<T>::kMinimalVectorDimension;
  }

  //! for a vector type
  template <int IsVector = TypeTraits<T>::kIsVector,
            int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsVector != 0 && IsScalar == 0, int>::type
  minimalDimension(const T& t) {
    return t.size();
  }

  //! for a scalar value
  template <int IsScalar = TypeTraits<T>::kIsScalar>
  static typename std::enable_if<IsScalar != 0, int>::type minimalDimension(
      const T&) {
    return 1;
  }
};

}  // namespace g2o

#endif
