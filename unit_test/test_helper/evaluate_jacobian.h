// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef EVALUATE_JACOBIAN_H
#define EVALUATE_JACOBIAN_H

#include <gmock/gmock.h>

#include <Eigen/Core>
#include <functional>

#include "eigen_matcher.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/eigen_types.h"

namespace g2o {

using EpsilonFunction = std::function<double(const double, const double)>;

namespace internal {
static EpsilonFunction epsilon = [](const double, const double) {
  return 1e-6;
};

/**
 * @brief Matcher for testing Numeric and Analytic Jacobian to be approximately
 * equal.
 */
MATCHER_P2(JacobianApproxEqual, expect, prec,
           std::string(negation ? "isn't" : "is") + " approx equal to" +
               ::testing::PrintToString(expect)) {
  if (arg.size() != expect.size()) return false;
  for (int j = 0; j < arg.size(); ++j) {
    double diff = std::abs(arg(j) - expect(j));
    if (diff > prec(arg(j), expect(j))) return false;
  }
  return true;
}
}  // namespace internal

template <typename EdgeType>
void evaluateJacobianUnary(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                           JacobianWorkspace& numericJacobianWorkspace,
                           const EpsilonFunction& eps = internal::epsilon) {
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseUnaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                  typename EdgeType::VertexXiType>::
      linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseUnaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                  typename EdgeType::VertexXiType>::linearizeOplus();

  // compare the Jacobians
  int numElems = EdgeType::kDimension;
  numElems *= EdgeType::VertexXiType::kDimension;
  VectorX::ConstMapType n(numericJacobianWorkspace.workspaceForVertex(0),
                          numElems);
  VectorX::ConstMapType a(jacobianWorkspace.workspaceForVertex(0), numElems);
  EXPECT_THAT(internal::print_wrap(a),
              internal::JacobianApproxEqual(internal::print_wrap(n), eps));
}

template <typename EdgeType>
void evaluateJacobian(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                      JacobianWorkspace& numericJacobianWorkspace,
                      const EpsilonFunction& eps = internal::epsilon) {
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseBinaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::
      linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseBinaryEdge<EdgeType::kDimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus();

  // compare the two Jacobians
  for (int i = 0; i < 2; ++i) {
    int numElems = EdgeType::kDimension;
    numElems *= i == 0 ? EdgeType::VertexXiType::kDimension
                       : EdgeType::VertexXjType::kDimension;
    VectorX::ConstMapType n(numericJacobianWorkspace.workspaceForVertex(i),
                            numElems);
    VectorX::ConstMapType a(jacobianWorkspace.workspaceForVertex(i), numElems);
    EXPECT_THAT(internal::print_wrap(a),
                internal::JacobianApproxEqual(internal::print_wrap(n), eps));
  }
}

}  // namespace g2o

#endif
