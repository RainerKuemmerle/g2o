// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_BASE_FIXED_SIZED_EDGE_H
#define G2O_BASE_FIXED_SIZED_EDGE_H

#include <array>
#include <iostream>
#include <limits>
#include <utility>

#ifdef G2O_USE_VENDORED_CERES
#include "g2o/EXTERNAL/ceres/fixed_array.h"
#else
#include <ceres/internal/fixed_array.h>
#endif

#include "base_edge.h"
#include "g2o/config.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/tuple_tools.h"
#include "robust_kernel.h"

namespace g2o {

namespace internal {

// creating a bool array
template <int K>
std::array<bool, K> createBoolArray() {
  std::array<bool, K> aux = {false};
  return aux;
}
template <>
inline std::array<bool, 0> createBoolArray<0>() {
  return std::array<bool, 0>();
}

// assumes i < j
// duplication of internal::computeUpperTriangleIndex in
// g2o/core/base_variable_sized_edge.hpp
constexpr int pair_to_index(const int i, const int j) {
  return j * (j - 1) / 2 + i;
}

/**
 * A trivial pair that has a constexpr c'tor.
 * std::pair in C++11 has not a constexpr c'tor.
 */
struct TrivialPair {
  int first, second;
  constexpr TrivialPair(int f, int s) : first(f), second(s) {}
  bool operator==(const TrivialPair& other) const {
    return first == other.first && second == other.second;
  }
};

/**
 * If we would have a constexpr for sqrt (cst_sqrt) it would be sth like.
 * For now we implement as a recursive function at compile time.
 * constexpr TrivialPair index_to_pair(n) {
 *   constexpr int j = int(0.5 + cst_sqrt(0.25 + 2 * n));
 *   constexpr int base = pair_to_index(0, j);
 *   constexpr int i = n - base;
 *   return TrivialPair(i, j);
 * }
 */
constexpr TrivialPair index_to_pair(const int k, const int j = 0) {
  return k < j ? TrivialPair{k, j} : index_to_pair(k - j, j + 1);
}

//! helper function to call the c'tor of Eigen::Map
template <typename T>
T createHessianMapK() {
  // if the size is known at compile time, we have to call the c'tor of
  // Eigen::Map with corresponding values
  constexpr int r =
      T::RowsAtCompileTime == Eigen::Dynamic ? 0 : T::RowsAtCompileTime;
  constexpr int c =
      T::ColsAtCompileTime == Eigen::Dynamic ? 0 : T::ColsAtCompileTime;
  return T(nullptr, r, c);
}
//! helper function for creating a tuple of Eigen::Map
template <typename... Args>
std::tuple<Args...> createHessianMaps(const std::tuple<Args...>&) {
  return std::tuple<Args...>{createHessianMapK<Args>()...};
}

// clang-format off
template <std::size_t I, typename... Tp>
typename std::enable_if<I >= sizeof...(Tp), OptimizableGraph::Vertex*>::type createNthVertexType(size_t) {
  return nullptr;  // end of recursion, return null
}
template <std::size_t I, typename... Tp>
typename std::enable_if <I < sizeof...(Tp), OptimizableGraph::Vertex*>::type createNthVertexType(size_t i) {
  if (i == I) {
    using VertexType = typename std::tuple_element<I, std::tuple<Tp...>>::type;
    return new VertexType();
  } else
    return createNthVertexType<I + 1, Tp...>(i);
}
// clang-format on
}  // namespace internal

template <int D, typename E, typename... VertexTypes>
class BaseFixedSizedEdge : public BaseEdge<D, E> {
 public:
  template <int N, typename... Types>
  using NthType = typename std::tuple_element<N, std::tuple<Types...>>::type;
  //! The type of the N-th vertex
  template <int VertexN>
  using VertexXnType = NthType<VertexN, VertexTypes...>;
  //! Size of the N-th vertex at compile time
  template <int VertexN>
  static constexpr int VertexDimension() {
    return VertexXnType<VertexN>::Dimension;
  };
  /**
   * Get the size of a given Vertex.
   * If the vertex dimension is static and by this known at compile time, we
   * return this. Otherwise we get the size at runtime.
   */
  // clang-format off
  template <int VertexN>
  constexpr typename std::enable_if<VertexXnType<VertexN>::Dimension != -1, int>::type vertexDimension() const {
    return VertexXnType<VertexN>::Dimension;
  };
  template <int VertexN>
  typename std::enable_if<VertexXnType<VertexN>::Dimension == -1, int>::type vertexDimension() const {
    return vertexXn<VertexN>()->dimension();
  };
  // clang-format on
  /**
   * Return a pointer to the N-th vertex, directly casted to the correct type
   */
  template <int VertexN>
  const VertexXnType<VertexN>* vertexXn() const {
    return static_cast<const VertexXnType<VertexN>*>(_vertices[VertexN]);
  }
  template <int VertexN>
  VertexXnType<VertexN>* vertexXn() {
    return static_cast<VertexXnType<VertexN>*>(_vertices[VertexN]);
  }

  static const int Dimension = BaseEdge<D, E>::Dimension;
  typedef typename BaseEdge<D, E>::Measurement Measurement;
  typedef typename BaseEdge<D, E>::ErrorVector ErrorVector;
  typedef typename BaseEdge<D, E>::InformationType InformationType;

  template <int EdgeDimension, int VertexDimension>
  using JacobianType = typename Eigen::Matrix<
      number_t, EdgeDimension, VertexDimension,
      EdgeDimension == 1 ? Eigen::RowMajor : Eigen::ColMajor>::AlignedMapType;

  //! it requires quite some ugly code to get the type of hessians...
  template <int DN, int DM>
  using HessianBlockType = Eigen::Map<
      Eigen::Matrix<number_t, DN, DM,
                    DN == 1 ? Eigen::RowMajor : Eigen::ColMajor>,
      Eigen::Matrix<number_t, DN, DM,
                    DN == 1 ? Eigen::RowMajor : Eigen::ColMajor>::Flags &
              Eigen::PacketAccessBit
          ? Eigen::Aligned
          : Eigen::Unaligned>;
  template <int K>
  using HessianBlockTypeK = HessianBlockType<
      VertexXnType<internal::index_to_pair(K).first>::Dimension,
      VertexXnType<internal::index_to_pair(K).second>::Dimension>;
  template <int K>
  using HessianBlockTypeKTransposed = HessianBlockType<
      VertexXnType<internal::index_to_pair(K).second>::Dimension,
      VertexXnType<internal::index_to_pair(K).first>::Dimension>;
  template <typename>
  struct HessianTupleType;
  template <std::size_t... Ints>
  struct HessianTupleType<std::index_sequence<Ints...>> {
    using type = std::tuple<HessianBlockTypeK<Ints>...>;
    using typeTransposed = std::tuple<HessianBlockTypeKTransposed<Ints>...>;
  };
  static const std::size_t _nr_of_vertices = sizeof...(VertexTypes);
  static const std::size_t _nr_of_vertex_pairs =
      internal::pair_to_index(0, _nr_of_vertices);
  using HessianTuple = typename HessianTupleType<
      std::make_index_sequence<_nr_of_vertex_pairs>>::type;
  using HessianTupleTransposed = typename HessianTupleType<
      std::make_index_sequence<_nr_of_vertex_pairs>>::typeTransposed;
  using HessianRowMajorStorage = std::array<bool, _nr_of_vertex_pairs>;

  BaseFixedSizedEdge()
      : BaseEdge<D, E>(),
        _hessianRowMajor(internal::createBoolArray<_nr_of_vertex_pairs>()),
        _hessianTuple(internal::createHessianMaps(_hessianTuple)),
        _hessianTupleTransposed(
            internal::createHessianMaps(_hessianTupleTransposed)),
        _jacobianOplus({nullptr, D, VertexTypes::Dimension}...) {
    _vertices.resize(_nr_of_vertices, nullptr);
  }

  //! create an instance of the Nth VertexType
  virtual OptimizableGraph::Vertex* createVertex(int i) {
    if (i < 0) return nullptr;
    return internal::createNthVertexType<0, VertexTypes...>(
        static_cast<size_t>(i));
  };

  virtual void resize(size_t size);

  template <std::size_t... Ints>
  bool allVerticesFixedNs(std::index_sequence<Ints...>) const;
  virtual bool allVerticesFixed() const;

  virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);
  template <std::size_t... Ints>
  void linearizeOplus_allocate(JacobianWorkspace& jacobianWorkspace,
                               std::index_sequence<Ints...>);

  /**
   * Linearizes the oplus operator in the vertex, and stores
   * the result in temporary variables _jacobianOplus
   */
  virtual void linearizeOplus();
  template <std::size_t... Ints>
  void linearizeOplusNs(std::index_sequence<Ints...>);
  template <int N>
  void linearizeOplusN();

  //! returns the result of the linearization in the manifold space for the
  //! nodes xn
  template <int N>
  const typename std::tuple_element<
      N, std::tuple<JacobianType<D, VertexTypes::Dimension>...>>::type&
  jacobianOplusXn() const {
    return std::get<N>(_jacobianOplus);
  }
  //! returns the result of the linearization in the manifold space for the
  //! nodes xn
  template <int N>
  typename std::tuple_element<
      N, std::tuple<JacobianType<D, VertexTypes::Dimension>...>>::type&
  jacobianOplusXn() {
    return std::get<N>(_jacobianOplus);
  }

  /**
   * computes the (block) elements of the Hessian matrix of the linearized least
   * squares.
   */
  virtual void constructQuadraticForm();
  template <std::size_t... Ints>
  void constructQuadraticFormNs(const InformationType& omega,
                                const ErrorVector& weightedError,
                                std::index_sequence<Ints...>);
  template <int N>
  void constructQuadraticFormN(const InformationType& omega,
                               const ErrorVector& weightedError);

  template <int N, typename AtOType>
  void constructOffDiagonalQuadraticFormMs(const AtOType&,
                                           std::index_sequence<>);

  template <int N, std::size_t... Ints, typename AtOType>
  void constructOffDiagonalQuadraticFormMs(const AtOType& AtO,
                                           std::index_sequence<Ints...>);
  template <int N, int M, typename AtOType>
  void constructOffDiagonalQuadraticFormM(const AtOType& AtO);

  virtual void mapHessianMemory(number_t* d, int i, int j, bool rowMajor);

  using BaseEdge<D, E>::resize;
  using BaseEdge<D, E>::computeError;

 protected:
  using BaseEdge<D, E>::_measurement;
  using BaseEdge<D, E>::_information;
  using BaseEdge<D, E>::_error;
  using BaseEdge<D, E>::_vertices;
  using BaseEdge<D, E>::_dimension;

  HessianRowMajorStorage _hessianRowMajor;
  HessianTuple _hessianTuple;
  HessianTupleTransposed _hessianTupleTransposed;
  std::tuple<JacobianType<D, VertexTypes::Dimension>...> _jacobianOplus;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_fixed_sized_edge.hpp"

}  // end namespace g2o

#endif
