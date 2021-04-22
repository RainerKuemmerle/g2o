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

#ifdef G2O_USE_VENDORED_CERES
  #include "g2o/EXTERNAL/ceres/autodiff.h"
#else
  #include <ceres/internal/autodiff.h>
#endif

#include "eigen_types.h"
#include "g2o/stuff/misc.h"
#include "g2o_core_api.h"

namespace g2o {

/**
 * functor object to access the estimate data of an edge.
 * Here, we call estimate().data() on each vertex to obtain the raw pointer.
 */
template <typename Edge>
struct EstimateAccessor {
  template <int k>
  EIGEN_STRONG_INLINE number_t* data(Edge* that) {
    return const_cast<number_t*>(that->template vertexXn<k>()->estimate().data());
  }
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
  template <int k>
  EIGEN_STRONG_INLINE number_t* data(Edge* that) {
    auto& buffer = std::get<k>(_estimateBuffer);
    buffer.resize(that->template vertexDimension<k>());
    number_t* rawBuffer = const_cast<number_t*>(buffer.data());
    bool gotData = that->template vertexXn<k>()->getEstimateData(rawBuffer);
    assert(gotData && "Called getEstimateData, but seems unimplmented");
    return gotData ? rawBuffer : nullptr;
  }

 protected:
  template <typename>
  struct BufferType;
  template <std::size_t... Ints>
  struct BufferType<std::index_sequence<Ints...>> {
    using type = std::tuple<VectorN<Edge::template VertexXnType<Ints>::Dimension>...>;
  };

  using Buffer = typename BufferType<std::make_index_sequence<Edge::_nr_of_vertices>>::type;
  Buffer _estimateBuffer;
};

/**
 * \brief Implementation of Automatic Differentiation for edges in g2o
 *
 * This class implements an interface to Automatic Differentiation, see, for example,
 * https://en.wikipedia.org/wiki/Automatic_differentiation for the idea behind it.
 *
 * Pre-condition:
 * Your estimate type in your vertices provides a method called data() which returns a raw pointer
 * to the data representing the estimate. This can, for example, be achieved by using Eigen's
 * vector underneath as the container for the data. An SE2 vertex might look as follows
 *
 * class VertexFlatSE2 : public g2o::BaseVertex<3, g2o::Vector3> {
 *  public:
 *   virtual void setToOriginImpl() { _estimate.setZero(); }
 *   virtual void oplusImpl(const number_t* update) {
 *    _estimate += Eigen::Map<const g2o::Vector3>(update);
 *    _estimate(2) = g2o::normalize_theta(_estimate(2));
 *   }
 *   virtual bool read(std::istream&) { return false; }
 *   virtual bool write(std::ostream&) const { return false; }
 * };
 *
 * If this is not the case for your edge, then you can provide a functor object as second template
 * argument which does this conversion for you. See EstimateAccessor above.
 * Such a functor has to provide a templatized function data(Edge*) that returns the raw-pointer to
 * the underlying data. The raw-pointer should point to memory that is either owned by the functor
 * itself or is owned by the edge, the vertex, or sth else. It has to to be valid throughout the
 * lifetime of the functor object.
 * See, for example, the functor EstimateAccessorGet which uses the potentially implemented
 * method getEstimateData() on vertices to obtain the estimate in a raw array. This array is then
 * buffered and passed on to compute the error or its Jacobian.
 *
 * To use automatic differentiation on your own edge you need to implement the following steps:
 * 1. Implement an operator() that computes your error function:
 *    The function is required to have the following declaration
 *    template <typename T>
 *    bool operator()(const T* v1Estimate, const T* v2Estimate, T* error) const {}
 *    The example above assumes a binary edge. If your edge has more or less vertices, the number
 *    of vEstimate parameters differs.
 *    Let's assume that your edge connects N vertices, then your operator() will consume N+1
 *    pointers. Whereas the last pointer is the output of your error function.
 *    Note the template on the operator(). This is required to be able to evaluate your error
 *    function with double pointer, i.e., to purely evaluate the error. But also we will pass
 *    a more complex class to it during the numerical computation of the Jacobian.
 * 2. Integrate the operator():
 *    To this end, we provide the macro "G2O_MAKE_AUTO_AD_FUNCTIONS" which you can include
 *    into the public section of your edge class. See below for the macro.
 *    If you use the macro, you do not need to implement computeError() and linearizeOPlus()
 *    in your edge. Both methods will be ready for integration into the g2o framework.
 *    You may, however, decide agains the macro and provide the implementation on your own if this
 *    suits your edge class better.
 *
 * Example integration: g2o/examples/bal/bal_example.cpp
 * This provides a self-contained example for integration of AD into an optimization problem.
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
      typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, Eigen::RowMajor>;

  //! helper for computing the error based on the functor in the edge
  static void computeError(Edge* that) {
    static_assert(Edge::Dimension > 0, "Dynamically sized edges are not supported");
    computeErrorNs(that, std::make_index_sequence<Edge::_nr_of_vertices>());
  }

  /**
   * Linearize (compute the Jacobians) for the given edge.
   * Stores the Jacobians in the members of the edge.
   * A vertex that is fixed will obtain a Jacobian with all elements set to zero.
   * In the particular case that all vertices are fixed, we terminate early and do not start
   * evaluation of the Jacobian.
   */
  static void linearize(Edge* that) {
    static_assert(Edge::Dimension > 0, "Dynamically sized edges are not supported");
    linearizeOplusNs(that, std::make_index_sequence<Edge::_nr_of_vertices>());
  }

 protected:
  //! packed version to call the functor that evaluates the error function
  template <std::size_t... Ints>
  static void computeErrorNs(Edge* that, std::index_sequence<Ints...>) {
    static_assert(std::min({Edge::template VertexXnType<Ints>::Dimension...}) > 0,
                  "Dynamically sized vertices are not supported");
    EstimateAccess estimateAccess;
    (*that)(estimateAccess.template data<Ints>(that)..., that->errorData());
  }

  /**
   * packed version of the code to linearize using AD
   */
  template <std::size_t... Ints>
  static void linearizeOplusNs(Edge* that, std::index_sequence<Ints...>) {
    static_assert(std::min({Edge::template VertexXnType<Ints>::Dimension...}) > 0,
                  "Dynamically sized vertices are not supported");
    // all vertices are fixed, no need to compute anything here
    if (that->allVerticesFixed()) {
      int unused[] = {(that->template jacobianOplusXn<Ints>().setZero(), 0)...};
      (void)unused;
      return;
    }

    // tuple containing the Jacobians
    std::tuple<ADJacobianType<Edge::Dimension, Edge::template VertexXnType<Ints>::Dimension>...>
        ad_jacobians;

    // setting up the pointer to the parameters and the Jacobians for calling AD.
    EstimateAccess estimateAccess;
    number_t* parameters[] = {estimateAccess.template data<Ints>(that)...};
    // number_t* parameters[] = { /* trivial case would be */
    //     const_cast<number_t*>(that->template vertexXn<Ints>()->estimate().data())...};

    // pointers to the Jacobians, set to NULL if vertex is fixed to skip computation
    number_t* jacobians[] = {that->template vertexXn<Ints>()->fixed()
                                 ? nullptr
                                 : const_cast<number_t*>(std::get<Ints>(ad_jacobians).data())...};
    // Calls the automatic differentiation for evaluation of the Jacobians.
    number_t errorValue[Edge::Dimension];
    using AutoDiffDims =
        ceres::internal::StaticParameterDims<Edge::template VertexXnType<Ints>::Dimension...>;
    bool diffState =
        ceres::internal::AutoDifferentiate<Edge::Dimension, AutoDiffDims, Edge, number_t>(
            *that, parameters, Edge::Dimension, errorValue, jacobians);

    assert(diffState && "Error during Automatic Differentiation");
    if (!diffState) {  // something went wrong during AD
      int unused[] = {(std::get<Ints>(ad_jacobians).setZero(), 0)...};
      (void)unused;
      return;
    }
    // copy over the Jacobians (convert row-major -> column-major) for non-fixed vertices
    int unused[] = {
        that->template vertexXn<Ints>()->fixed()
            ? (that->template jacobianOplusXn<Ints>().setZero(), 0)
            : (assign(that->template jacobianOplusXn<Ints>(), std::get<Ints>(ad_jacobians)), 0)...};
    (void)unused;
  }

  //! helper function to perform a = b
  template <typename A, typename B>
  static EIGEN_STRONG_INLINE void assign(const Eigen::MatrixBase<A>& a,
                                         const Eigen::MatrixBase<B>& b) {
    Eigen::MatrixBase<A>& aux = const_cast<Eigen::MatrixBase<A>&>(a);
    aux = b;
  }
};

}  // namespace g2o

// helper macros for fine-grained integration into own types
#define G2O_MAKE_AUTO_AD_COMPUTEERROR                                                           \
  void computeError() {                                                                         \
    g2o::AutoDifferentiation<std::remove_reference<decltype(*this)>::type>::computeError(this); \
  }
#define G2O_MAKE_AUTO_AD_LINEARIZEOPLUS                                                      \
  void linearizeOplus() {                                                                    \
    g2o::AutoDifferentiation<std::remove_reference<decltype(*this)>::type>::linearize(this); \
  }

/**
 * Helper macro for easy integration into own types
 */
#define G2O_MAKE_AUTO_AD_FUNCTIONS \
  G2O_MAKE_AUTO_AD_COMPUTEERROR    \
  G2O_MAKE_AUTO_AD_LINEARIZEOPLUS

// helper macros for fine-grained integration into own types using EstimateAccessorGet
#define G2O_MAKE_AUTO_AD_COMPUTEERROR_BY_GET                                                    \
  void computeError() {                                                                         \
    using EdgeType = std::remove_reference<decltype(*this)>::type;                              \
    g2o::AutoDifferentiation<EdgeType, g2o::EstimateAccessorGet<EdgeType>>::computeError(this); \
  }

#define G2O_MAKE_AUTO_AD_LINEARIZEOPLUS_BY_GET                                               \
  void linearizeOplus() {                                                                    \
    using EdgeType = std::remove_reference<decltype(*this)>::type;                           \
    g2o::AutoDifferentiation<EdgeType, g2o::EstimateAccessorGet<EdgeType>>::linearize(this); \
  }

/**
 * Helper macro for easy integration into own types
 */
#define G2O_MAKE_AUTO_AD_FUNCTIONS_BY_GET \
  G2O_MAKE_AUTO_AD_COMPUTEERROR_BY_GET    \
  G2O_MAKE_AUTO_AD_LINEARIZEOPLUS_BY_GET

#endif
