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

#include <type_traits>

#include "eigen_types.h"
#include "g2o/EXTERNAL/ceres/autodiff.h"
#include "g2o/stuff/misc.h"
#include "g2o_core_api.h"

namespace g2o {

/**
 * \brief Implementation of Automatic Differentiation for edges in g2o
 *
 * This class implements an interface to Automatic Differentiation, see, for example,
 * https://en.wikipedia.org/wiki/Automatic_differentiation for the idea behind it.
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
 *    To this end, we provide the macro "G20_MAKE_AUTO_AD_FUNCTIONS" which you can include
 *    into the public section of your edge class. See below for the macro.
 *    If you use the macro, you do not need to implement computeError() and linearizeOPlus()
 *    in your edge. Both methods will be ready for integration into the g2o framework.
 *
 * Example integration: g2o/examples/bal/bal_example.cpp
 * This provides a self-contained example for integration of AD into an optimization problem.
 *
 * Further documentation on the underlying implementation:
 * Jet: EXTERNAL/ceres/jet.h
 * AutoDiff: EXTERNAL/ceres/autodiff.h
 */
template <typename Edge>
class AutoDifferentiation {
 public:
  //! type for the Jacobians during AD
  template <int EdgeDimension, int VertexDimension>
  using ADJacobianType =
      typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, Eigen::RowMajor>;

  //! helper for computing the error based on the functor in the edge
  void computeError(Edge* that) {
    computeErrorNs(that, make_index_sequence<Edge::_nr_of_vertices>());
  }

  /**
   * Linearize (compute the Jacobians) for the given edge.
   * Stores the Jacobians in the members of the edge.
   */
  void linearize(Edge* that) {
    linearizeOplusNs(that, make_index_sequence<Edge::_nr_of_vertices>());
  }

 protected:
  //! packed version to call the functor that evaluates the error function
  template <std::size_t... Ints>
  void computeErrorNs(Edge* that, index_sequence<Ints...>) {
    (*that)(that->template vertexXn<Ints>()->estimate().data()..., that->error().data());
  }

  /**
   * packed version of the code to linearize using AD
   */
  template <std::size_t... Ints>
  void linearizeOplusNs(Edge* that, index_sequence<Ints...>) {
    // all vertices are fixed, no need to compute anything here
    if (that->allVerticesFixed()) return;

    // tuple containing the Jacobians
    std::tuple<ADJacobianType<Edge::Dimension, Edge::template VertexXnType<Ints>::Dimension>...>
        ad_jacobians;

    // setting up the pointer to the parameters and the Jacobians for calling AD.
    number_t* parameters[] = {
        const_cast<number_t*>(that->template vertexXn<Ints>()->estimate().data())...};
    // pointers to the Jacobians, set to NULL if vertex is fixed
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
  EIGEN_STRONG_INLINE void assign(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b) {
    Eigen::MatrixBase<A>& aux = const_cast<Eigen::MatrixBase<A>&>(a);
    aux = b;
  }
};

}  // namespace g2o

/**
 * Helper macro for easy integration into own types
 */
#define G20_MAKE_AUTO_AD_FUNCTIONS                                                               \
  void computeError() {                                                                          \
    g2o::AutoDifferentiation<std::remove_reference<decltype(*this)>::type>().computeError(this); \
  }                                                                                              \
  void linearizeOplus() {                                                                        \
    g2o::AutoDifferentiation<std::remove_reference<decltype(*this)>::type>().linearize(this);    \
  }

#endif
