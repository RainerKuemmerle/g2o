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

#ifndef G2O_AUTO_DIFFERENTIATION_H
#define G2O_AUTO_DIFFERENTIATION_H

#include <algorithm>
#include <cassert>
#include <type_traits>

#include "eigen_types.h"
#include "g2o/autodiff/autodiff.h"
#include "g2o/core/type_traits.h"

namespace g2o {

namespace internal {
template <int...>
struct IntPack;
}  // namespace internal

/**
 * functor object to access the estimate data of an edge.
 * Here, we call estimate().data() if EstimateType is a vector or convert to
 * vector on the fly otherwise on each vertex to obtain the raw pointer.
 */
template <typename Edge>
class EstimateAccessor {
 public:
  //! VertexXnType's EstimateType is a vector type
  template <int K>
  typename std::enable_if<TypeTraits<typename Edge::template VertexXnType<
                              K>::EstimateType>::kIsVector != 0,
                          double*>::type
  data(Edge* that) {
    return const_cast<double*>(that->template vertexXn<K>()->estimate().data());
  }

  /**
   * VertexXnType's EstimateType is not a vector type. In this fallback method,
   * we buffer the estimates into a unordered map.
   */
  template <int K>
  typename std::enable_if<TypeTraits<typename Edge::template VertexXnType<
                              K>::EstimateType>::kIsVector == 0,
                          double*>::type
  data(Edge* that) {
    VectorX& data_buffer = buffer_[K];
    if (data_buffer.size() > 0) {  // trivial caching
      return data_buffer.data();
    }
    data_buffer =
        TypeTraits<typename Edge::template VertexXnType<K>::EstimateType>::
            toVector(that->template vertexXn<K>()->estimate());
    return data_buffer.data();
  }

 private:
  //! An empty struct
  struct EmptyBuffer {};
  //! Array for storing the estimates as vector
  using Buffer = std::array<VectorX, Edge::kNrOfVertices>;

  template <typename>
  struct AnyNonVectorEstimateTypePack;

  template <std::size_t... Ints>
  struct AnyNonVectorEstimateTypePack<std::index_sequence<Ints...>> {
    // kValue = true iff kIsVector == 0 for all EstimateType
    static constexpr bool kValue = std::is_same<
        internal::IntPack<TypeTraits<typename Edge::template VertexXnType<
                              Ints>::EstimateType>::kIsVector...,
                          0>,
        internal::IntPack<0, TypeTraits<typename Edge::template VertexXnType<
                                 Ints>::EstimateType>::kIsVector...>>::value;
  };
  static constexpr bool kAnyNonVectorEstimateType =
      AnyNonVectorEstimateTypePack<
          std::make_index_sequence<Edge::kNrOfVertices>>::kValue;

  // An array for buffering the data or an empty struct
  using StorageType = typename std::conditional_t<kAnyNonVectorEstimateType,
                                                  Buffer, EmptyBuffer>;
  StorageType buffer_;
};

/**
 * functor to access the estimate pointer by the getEstimateData() function
 * that can be implemented in vertex types. Downside is that we have to
 * buffer the data.
 * In such a case, Edge::computeError is most likely implemented on own types
 * and Edge::operator() has to be re-implemented consuming pointers to obtain
 * the error for linearizing.
 * See the implementation of EdgeSE2AD in unit_test/general/auto_diff.cpp
 */
template <typename Edge>
class EstimateAccessorGet {
 public:
  template <int K>
  double* data(Edge* that) {
    auto& buffer = std::get<K>(estimateBuffer_);
    buffer.resize(that->template vertexDimension<K>());
    auto* rawBuffer = const_cast<double*>(buffer.data());
    bool gotData = that->template vertexXn<K>()->getEstimateData(rawBuffer);
    assert(gotData && "Called getEstimateData, but seems unimplemented");
    return gotData ? rawBuffer : nullptr;
  }

 protected:
  template <typename>
  struct BufferType;
  template <std::size_t... Ints>
  struct BufferType<std::index_sequence<Ints...>> {
    using type =
        std::tuple<VectorN<Edge::template VertexXnType<Ints>::kDimension>...>;
  };

  using Buffer =
      typename BufferType<std::make_index_sequence<Edge::kNrOfVertices>>::type;
  Buffer estimateBuffer_;
};

/**
 * \brief Implementation of Automatic Differentiation for edges in g2o
 *
 * This class implements an interface to Automatic Differentiation, see, for
 * example, https://en.wikipedia.org/wiki/Automatic_differentiation for the idea
 * behind it.
 *
 * Pre-condition:
 * Your estimate type in your vertices provides a method called data() which
 * returns a raw pointer to the data representing the estimate. This can, for
 * example, be achieved by using Eigen's vector underneath as the container for
 * the data. An SE2 vertex might look as follows
 *
 * class VertexFlatSE2 : public g2o::BaseVertex<3, g2o::Vector3> {
 *  public:
 *   virtual void oplusImpl(const double* update) {
 *    _estimate += Eigen::Map<const g2o::Vector3>(update);
 *    _estimate(2) = g2o::normalize_theta(_estimate(2));
 *   }
 * };
 *
 * If this is not the case for your edge, then you can provide a functor object
 * as second template argument which does this conversion for you. See
 * EstimateAccessor above. Such a functor has to provide a templatized function
 * data(Edge*) that returns the raw-pointer to the underlying data. The
 * raw-pointer should point to memory that is either owned by the functor itself
 * or is owned by the edge, the vertex, or sth else. It has to to be valid
 * throughout the lifetime of the functor object. See, for example, the functor
 * EstimateAccessorGet which uses the type traits to convert the estimate of the
 * vertices into a raw array. This array is then buffered and passed on to
 * compute the error or its Jacobian. Note that EstimateAccessor provides a
 * fallback to also support a mix of get and access to raw-pointers.
 *
 * To use automatic differentiation on your own edge you need to implement the
 * following steps:
 * 1. Implement an operator() that computes your error function:
 *    The function is required to have the following declaration
 *    template <typename T>
 *    bool operator()(const T* v1Estimate, const T* v2Estimate, T* error) const;
 *    The example above assumes a binary edge. If your edge has more or less
 *    vertices, the number of vEstimate parameters differs. Let's assume that
 *    your edge connects N vertices, then your operator() will consume N+1
 *    pointers. Whereas the last pointer is the output of your error function.
 *    Note the template on the operator(). This is required to be able to
 *    evaluate your error function with double pointer, i.e., to purely evaluate
 *    the error. But also we will pass a more complex class to it during the
 *    numerical computation of the Jacobian.
 * 2. Integrate the operator():
 *    To this end, we provide the macro "G2O_MAKE_AUTO_AD_FUNCTIONS" which you
 *    can include into the public section of your edge class. See below for the
 *    macro. If you use the macro, you do not need to implement computeError()
 *    and linearizeOPlus() in your edge. Both methods will be ready for
 *    integration into the g2o framework. You may, however, decide against the
 *    macro and provide the implementation on your own if this suits your edge
 *    class better.
 *
 * Example integration: g2o/examples/bal/bal_example.cpp
 * This provides a self-contained example for integration of AD into an
 * optimization problem.
 *
 * Further documentation on the underlying implementation:
 * Jet: EXTERNAL/ceres/jet.h
 * AutoDiff: EXTERNAL/ceres/autodiff.h
 */
template <typename Edge, typename EstimateAccess = EstimateAccessor<Edge>>
class AutoDifferentiation {
 public:
  //! type for the Jacobians during AD
  template <int EdgeDimension, int VertexDimension>
  using ADJacobianType =
      typename Eigen::Matrix<double, EdgeDimension, VertexDimension,
                             (EdgeDimension > 1 && VertexDimension == 1)
                                 ? Eigen::ColMajor
                                 : Eigen::RowMajor>;

  //! helper for computing the error based on the functor in the edge
  static void computeError(Edge* that) {
    static_assert(Edge::kDimension > 0,
                  "Dynamically sized edges are not supported");
    computeErrorNs(that, std::make_index_sequence<Edge::kNrOfVertices>());
  }

  /**
   * Linearize (compute the Jacobians) for the given edge.
   * Stores the Jacobians in the members of the edge.
   * A vertex that is fixed will obtain a Jacobian with all elements set to
   * zero. In the particular case that all vertices are fixed, we terminate
   * early and do not start evaluation of the Jacobian.
   */
  static void linearize(Edge* that) {
    static_assert(Edge::kDimension > 0,
                  "Dynamically sized edges are not supported");
    linearizeOplusNs(that, std::make_index_sequence<Edge::kNrOfVertices>());
  }

 protected:
  //! packed version to call the functor that evaluates the error function
  template <std::size_t... Ints>
  static void computeErrorNs(Edge* that, std::index_sequence<Ints...>) {
    static_assert(
        std::min({Edge::template VertexXnType<Ints>::kDimension...}) > 0,
        "Dynamically sized vertices are not supported");
    EstimateAccess estimateAccess;
    (*that)(estimateAccess.template data<Ints>(that)..., that->errorData());
  }

  /**
   * packed version of the code to linearize using AD
   */
  template <std::size_t... Ints>
  static void linearizeOplusNs(Edge* that, std::index_sequence<Ints...>) {
    static_assert(
        std::min({Edge::template VertexXnType<Ints>::kDimension...}) > 0,
        "Dynamically sized vertices are not supported");
    // all vertices are fixed, no need to compute anything here
    if (that->allVerticesFixed()) {
      (void(that->template jacobianOplusXn<Ints>().setZero()), ...);
      return;
    }

    // tuple containing the Jacobians
    std::tuple<ADJacobianType<Edge::kDimension,
                              Edge::template VertexXnType<Ints>::kDimension>...>
        ad_jacobians;

    // setting up the pointer to the parameters and the Jacobians for calling
    // AD.
    EstimateAccess estimateAccess;
    double* parameters[] = {estimateAccess.template data<Ints>(that)...};
    // double* parameters[] = { /* trivial case would be */
    //     const_cast<double*>(that->template
    //     vertexXn<Ints>()->estimate().data())...};

    // pointers to the Jacobians, set to NULL if vertex is fixed to skip
    // computation
    double* jacobians[] = {
        that->template vertexXn<Ints>()->fixed()
            ? nullptr
            : const_cast<double*>(std::get<Ints>(ad_jacobians).data())...};
    // Calls the automatic differentiation for evaluation of the Jacobians.
    double errorValue[Edge::kDimension];
    using AutoDiffDims = ceres::internal::StaticParameterDims<
        Edge::template VertexXnType<Ints>::kDimension...>;
    bool diffState =
        ceres::internal::AutoDifferentiate<Edge::kDimension, AutoDiffDims, Edge,
                                           double>(
            *that, parameters, Edge::kDimension, errorValue, jacobians);

    assert(diffState && "Error during Automatic Differentiation");
    if (!diffState) {  // something went wrong during AD
      (void(std::get<Ints>(ad_jacobians).setZero()), ...);
      return;
    }
    // copy over the Jacobians (convert row-major -> column-major) for non-fixed
    // vertices
    (void(that->template vertexXn<Ints>()->fixed()
              ? (that->template jacobianOplusXn<Ints>().setZero(), 0)
              : (assign(that->template jacobianOplusXn<Ints>(),
                        std::get<Ints>(ad_jacobians)),
                 0)),
     ...);
  }

  //! helper function to perform a = b
  template <typename A, typename B>
  static EIGEN_STRONG_INLINE void assign(const Eigen::MatrixBase<A>& a,
                                         const Eigen::MatrixBase<B>& b) {
    auto& aux = const_cast<Eigen::MatrixBase<A>&>(a);
    aux = b;
  }
};

}  // namespace g2o

// helper macros for fine-grained integration into own types
#define G2O_MAKE_AUTO_AD_COMPUTEERROR                                  \
  void computeError() override {                                       \
    g2o::AutoDifferentiation<                                          \
        std::remove_reference_t<decltype(*this)>>::computeError(this); \
  }
#define G2O_MAKE_AUTO_AD_LINEARIZEOPLUS                             \
  void linearizeOplus() override {                                  \
    g2o::AutoDifferentiation<                                       \
        std::remove_reference_t<decltype(*this)>>::linearize(this); \
  }

/**
 * Helper macro for easy integration into own types
 */
#define G2O_MAKE_AUTO_AD_FUNCTIONS \
  G2O_MAKE_AUTO_AD_COMPUTEERROR    \
  G2O_MAKE_AUTO_AD_LINEARIZEOPLUS

// helper macros for fine-grained integration into own types using
// EstimateAccessorGet
#define G2O_MAKE_AUTO_AD_COMPUTEERROR_BY_GET                               \
  void computeError() override {                                           \
    using EdgeType = std::remove_reference_t<decltype(*this)>;             \
    g2o::AutoDifferentiation<                                              \
        EdgeType, g2o::EstimateAccessorGet<EdgeType>>::computeError(this); \
  }

#define G2O_MAKE_AUTO_AD_LINEARIZEOPLUS_BY_GET                          \
  void linearizeOplus() override {                                      \
    using EdgeType = std::remove_reference_t<decltype(*this)>;          \
    g2o::AutoDifferentiation<                                           \
        EdgeType, g2o::EstimateAccessorGet<EdgeType>>::linearize(this); \
  }

/**
 * Helper macro for easy integration into own types
 */
#define G2O_MAKE_AUTO_AD_FUNCTIONS_BY_GET \
  G2O_MAKE_AUTO_AD_COMPUTEERROR_BY_GET    \
  G2O_MAKE_AUTO_AD_LINEARIZEOPLUS_BY_GET

#endif
