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

#include <Eigen/Core>
#include <array>
#include <cassert>
#include <cstddef>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

#include "base_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/tuple_tools.h"
#include "robust_kernel.h"  // IWYU pragma: keep

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
constexpr TrivialPair index_to_pair(const int k, const int j = 0) {  // NOLINT
  return k < j ? TrivialPair{k, j} : index_to_pair(k - j, j + 1);    // NOLINT
}

//! helper function to call the c'tor of Eigen::Map
template <typename T>
T createHessianMapK() {
  // if the size is known at compile time, we have to call the c'tor of
  // Eigen::Map with corresponding values
  constexpr int kR =
      T::RowsAtCompileTime == Eigen::Dynamic ? 0 : T::RowsAtCompileTime;
  constexpr int kC =
      T::ColsAtCompileTime == Eigen::Dynamic ? 0 : T::ColsAtCompileTime;
  return T(nullptr, kR, kC);
}
//! helper function for creating a tuple of Eigen::Map
template <typename... Args>
std::tuple<Args...> createHessianMaps(const std::tuple<Args...>&) {
  return std::tuple<Args...>{createHessianMapK<Args>()...};
}

template <int I, typename EdgeType, typename... CtorArgs>
std::enable_if_t<I == -1, OptimizableGraph::Vertex*> createNthVertexType(
    int /*i*/, const EdgeType& /*t*/, CtorArgs... /*args*/) {
  return nullptr;
}

template <int I, typename EdgeType, typename... CtorArgs>
std::enable_if_t<I != -1, OptimizableGraph::Vertex*> createNthVertexType(
    int i, const EdgeType& t, CtorArgs... args) {
  if (i == I) {
    using VertexType = typename EdgeType::template VertexXnType<I>;
    return new VertexType(args...);
  }
  return createNthVertexType<I - 1, EdgeType, CtorArgs...>(i, t, args...);
}
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
    return VertexXnType<VertexN>::kDimension;
  }
  /**
   * Get the size of a given Vertex.
   * If the vertex dimension is static and by this known at compile time, we
   * return this. Otherwise we get the size at runtime.
   */
  // clang-format off
  template <int VertexN>
  constexpr  std::enable_if_t<VertexXnType<VertexN>::kDimension != -1, int> vertexDimension() const {
    return VertexXnType<VertexN>::kDimension;
  }
  template <int VertexN>
   std::enable_if_t<VertexXnType<VertexN>::kDimension == -1, int> vertexDimension() const {
    return vertexXn<VertexN>()->dimension();
  }
  // clang-format on
  /**
   * Return a pointer to the N-th vertex, directly casted to the correct type
   */
  template <int VertexN>
  std::shared_ptr<const VertexXnType<VertexN>> vertexXn() const {
    return std::static_pointer_cast<const VertexXnType<VertexN>>(
        vertices_[VertexN]);
  }
  template <int VertexN>
  std::shared_ptr<VertexXnType<VertexN>> vertexXn() {
    return std::static_pointer_cast<VertexXnType<VertexN>>(vertices_[VertexN]);
  }

  static const int kDimension = BaseEdge<D, E>::kDimension;
  using Measurement = typename BaseEdge<D, E>::Measurement;
  using ErrorVector = typename BaseEdge<D, E>::ErrorVector;
  using InformationType = typename BaseEdge<D, E>::InformationType;

  template <int EdgeDimension, int VertexDimension>
  using JacobianType = typename Eigen::Matrix<
      double, EdgeDimension, VertexDimension,
      EdgeDimension == 1 ? Eigen::RowMajor : Eigen::ColMajor>::AlignedMapType;

  //! it requires quite some ugly code to get the type of hessians...
  template <int DN, int DM>
  using HessianBlockType = Eigen::Map<
      Eigen::Matrix<double, DN, DM,
                    DN == 1 ? Eigen::RowMajor : Eigen::ColMajor>,
      Eigen::Matrix<double, DN, DM,
                    DN == 1 ? Eigen::RowMajor : Eigen::ColMajor>::Flags &
              Eigen::PacketAccessBit
          ? Eigen::Aligned
          : Eigen::Unaligned>;
  template <int K>
  using HessianBlockTypeK = HessianBlockType<
      VertexXnType<internal::index_to_pair(K).first>::kDimension,
      VertexXnType<internal::index_to_pair(K).second>::kDimension>;
  template <int K>
  using HessianBlockTypeKTransposed = HessianBlockType<
      VertexXnType<internal::index_to_pair(K).second>::kDimension,
      VertexXnType<internal::index_to_pair(K).first>::kDimension>;
  template <typename>
  struct HessianTupleType;

  template <std::size_t... Ints>
  struct HessianTupleType<std::index_sequence<Ints...>> {
    using type = std::tuple<HessianBlockTypeK<Ints>...>;
    using typeTransposed = std::tuple<HessianBlockTypeKTransposed<Ints>...>;
  };
  static constexpr std::size_t kNrOfVertices = sizeof...(VertexTypes);
  static constexpr std::size_t kNrOfVertexPairs =
      internal::pair_to_index(0, kNrOfVertices);
  using HessianTuple = typename HessianTupleType<
      std::make_index_sequence<kNrOfVertexPairs>>::type;
  using HessianTupleTransposed = typename HessianTupleType<
      std::make_index_sequence<kNrOfVertexPairs>>::typeTransposed;
  using HessianRowMajorStorage = std::array<bool, kNrOfVertexPairs>;

  BaseFixedSizedEdge()
      : BaseEdge<D, E>(),
        hessianRowMajor_(internal::createBoolArray<kNrOfVertexPairs>()),
        hessianTuple_(internal::createHessianMaps(hessianTuple_)),
        hessianTupleTransposed_(
            internal::createHessianMaps(hessianTupleTransposed_)),
        jacobianOplus_({nullptr, D, VertexTypes::kDimension}...) {
    vertices_.resize(kNrOfVertices, nullptr);
  }

  //! create an instance of the Nth VertexType
  template <typename... CtorArgs>
  OptimizableGraph::Vertex* createVertex(int i, CtorArgs... args) {
    if (i < 0) return nullptr;
    return internal::createNthVertexType<
        sizeof...(VertexTypes) - 1, std::remove_reference_t<decltype(*this)>,
        CtorArgs...>(i, *this, args...);
  }

  void resize(size_t size) override;

  template <std::size_t... Ints>
  bool allVerticesFixedNs(std::index_sequence<Ints...>) const;
  [[nodiscard]] bool allVerticesFixed() const override;

  void linearizeOplus(JacobianWorkspace& jacobianWorkspace) override;
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
      N, std::tuple<JacobianType<D, VertexTypes::kDimension>...>>::type&
  jacobianOplusXn() const {
    return std::get<N>(jacobianOplus_);
  }
  //! returns the result of the linearization in the manifold space for the
  //! nodes xn
  template <int N>
  typename std::tuple_element<
      N, std::tuple<JacobianType<D, VertexTypes::kDimension>...>>::type&
  jacobianOplusXn() {
    return std::get<N>(jacobianOplus_);
  }

  /**
   * computes the (block) elements of the Hessian matrix of the linearized least
   * squares.
   */
  void constructQuadraticForm() override;
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

  void mapHessianMemory(double* d, int i, int j, bool rowMajor) override;

  using BaseEdge<D, E>::resize;
  using BaseEdge<D, E>::computeError;

  [[nodiscard]] int numVerticesAtCompileTime() const final {
    return kNrOfVertices;
  }

 protected:
  using BaseEdge<D, E>::measurement_;
  using BaseEdge<D, E>::information_;
  using BaseEdge<D, E>::error_;
  using BaseEdge<D, E>::vertices_;
  using BaseEdge<D, E>::dimension_;

  HessianRowMajorStorage hessianRowMajor_;
  HessianTuple hessianTuple_;
  HessianTupleTransposed hessianTupleTransposed_;
  std::tuple<JacobianType<D, VertexTypes::kDimension>...> jacobianOplus_;

  /**
   * Only for use internally in sub-classes. It exposes the raw pointer for
   * implementation of, for example, computeError and other implementation of
   * function in the scope of a sub-class.
   */
  template <int VertexN>
  VertexXnType<VertexN>* vertexXnRaw() const {
    return static_cast<VertexXnType<VertexN>*>(vertices_[VertexN].get());
  }

 public:
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::resize(size_t size) {
  assert(size == kNrOfVertices && "attempting to resize a constant size edge");
  BaseEdge<D, E>::resize(size);
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::allVerticesFixedNs(
    std::index_sequence<Ints...>) const {
  return (... && vertexXn<Ints>()->fixed());
}

template <int D, typename E, typename... VertexTypes>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::allVerticesFixed() const {
  return allVerticesFixedNs(std::make_index_sequence<kNrOfVertices>());
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticForm() {
  if (this->robustKernel()) {
    double error = this->chi2();
    Vector3 rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<double, D, 1, Eigen::ColMajor> omega_r =
        -information_ * error_;
    omega_r *= rho[1];
    constructQuadraticFormNs(this->robustInformation(rho), omega_r,
                             std::make_index_sequence<kNrOfVertices>());
  } else {
    constructQuadraticFormNs(information_, -information_ * error_,
                             std::make_index_sequence<kNrOfVertices>());
  }
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticFormNs(
    const InformationType& omega, const ErrorVector& weightedError,
    std::index_sequence<Ints...>) {
  (void(constructQuadraticFormN<Ints>(omega, weightedError)), ...);
}

// overloading constructOffDiagonalQuadraticFormMs to
// prevent MSVC error when index_sequence is empty
template <int D, typename E, typename... VertexTypes>
template <int N, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormMs(const AtOType&, std::index_sequence<>) {
}

template <int D, typename E, typename... VertexTypes>
template <int N, std::size_t... Ints, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormMs(const AtOType& AtO,
                                        std::index_sequence<Ints...>) {
  (void(constructOffDiagonalQuadraticFormM<N, Ints, AtOType>(AtO)), ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N, int M, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::
    constructOffDiagonalQuadraticFormM(const AtOType& AtO) {
  constexpr auto kFromId = N;
  constexpr auto kToId = N + M + 1;
  assert(kFromId < kToId && "Index mixed up");
  auto to = vertexXn<kToId>();
  if (!to->fixed()) {
    const auto& B = std::get<kToId>(jacobianOplus_);
    constexpr auto kK = internal::pair_to_index(kFromId, kToId);
    internal::QuadraticFormLock lck(*to);
    (void)lck;
    if (hessianRowMajor_[kK]) {  // we have to write to the block as transposed
      auto& hessianTransposed = std::get<kK>(hessianTupleTransposed_);
      hessianTransposed.noalias() += B.transpose() * AtO.transpose();
    } else {
      auto& hessian = std::get<kK>(hessianTuple_);
      hessian.noalias() += AtO * B;
    }
  }
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::constructQuadraticFormN(
    const InformationType& omega, const ErrorVector& weightedError) {
  auto from = vertexXn<N>();
  const auto& A = std::get<N>(jacobianOplus_);

  if (!(from->fixed())) {
    const auto AtO = A.transpose() * omega;
    {
      internal::QuadraticFormLock lck(*from);
      (void)lck;
      from->b().noalias() += A.transpose() * weightedError;
      from->A().noalias() += AtO * A;
    }
    constructOffDiagonalQuadraticFormMs<N>(
        AtO, std::make_index_sequence<kNrOfVertices - N - 1>());
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus(
    JacobianWorkspace& jacobianWorkspace) {
  linearizeOplus_allocate(jacobianWorkspace,
                          std::make_index_sequence<kNrOfVertices>());
  linearizeOplus();
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus_allocate(
    JacobianWorkspace& jacobianWorkspace, std::index_sequence<Ints...>) {
  (new (&std::get<Ints>(jacobianOplus_))
       JacobianType<D, VertexDimension<Ints>()>(
           jacobianWorkspace.workspaceForVertex(Ints), D < 0 ? dimension_ : D,
           VertexDimension<Ints>() < 0 ? vertexXn<Ints>()->dimension()
                                       : VertexDimension<Ints>()),
   ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplusN() {
  auto vertex = vertexXn<N>();

  if (vertex->fixed()) return;

  auto& jacobianOplus = std::get<N>(jacobianOplus_);

  constexpr double kDelta = cst(1e-9);
  constexpr double kScalar = 1 / (2 * kDelta);

  internal::QuadraticFormLock lck(*vertex);
  (void)lck;

  using VectorType = typename VertexXnType<N>::BVector;
  VectorType add_vertex_buffer(vertexDimension<N>());
  add_vertex_buffer.fill(0.);
  VectorX::MapType add_vertex(add_vertex_buffer.data(),
                              add_vertex_buffer.size());

  // estimate the jacobian numerically
  // add small step along the unit vector in each dimension
  for (int d = 0; d < vertexDimension<N>(); ++d) {
    vertex->push();
    add_vertex[d] = kDelta;
    vertex->oplus(add_vertex);
    computeError();
    auto errorBak = this->error();
    vertex->pop();
    vertex->push();
    add_vertex[d] = -kDelta;
    vertex->oplus(add_vertex);
    computeError();
    errorBak -= this->error();
    vertex->pop();
    add_vertex[d] = 0.0;

    jacobianOplus.col(d) = kScalar * errorBak;
  }  // end dimension
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplusNs(
    std::index_sequence<Ints...>) {
  (void(linearizeOplusN<Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::linearizeOplus() {
  if (allVerticesFixed()) return;
  ErrorVector errorBeforeNumeric = error_;
  linearizeOplusNs(std::make_index_sequence<kNrOfVertices>());
  error_ = errorBeforeNumeric;
}

/**
 * Helper functor class to construct the Hessian Eigen::Map object.
 * We have to pass the size at runtime to allow dynamically sized verices.
 */
struct MapHessianMemoryK {
  double* d;
  int rows;
  int cols;
  template <typename HessianT>
  void operator()(HessianT& hessian) {
    new (&hessian)
        typename std::remove_reference<decltype(hessian)>::type(d, rows, cols);
  }
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::mapHessianMemory(double* d,
                                                                int i, int j,
                                                                bool rowMajor) {
  assert(i < j && "index assumption violated");
  // get the size of the vertices
  int vi_dim =
      static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(i).get())
          ->dimension();
  int vj_dim =
      static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(j).get())
          ->dimension();
  int k = internal::pair_to_index(i, j);
  hessianRowMajor_[k] = rowMajor;
  if (rowMajor)
    tuple_apply_i(MapHessianMemoryK{d, vj_dim, vi_dim}, hessianTupleTransposed_,
                  k);
  else
    tuple_apply_i(MapHessianMemoryK{d, vi_dim, vj_dim}, hessianTuple_, k);
}

}  // end namespace g2o

#endif
