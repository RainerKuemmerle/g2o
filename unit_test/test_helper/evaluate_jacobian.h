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

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"

#include <Eigen/Core>

#include <gtest/gtest.h>

namespace g2o {

template <typename EdgeType>
void evaluateJacobianUnary(EdgeType& e, JacobianWorkspace& jacobianWorkspace, JacobianWorkspace& numericJacobianWorkspace)
{
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseUnaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
    typename EdgeType::VertexXiType>::linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as setup by the previous call
  e.BaseUnaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
    typename EdgeType::VertexXiType>::linearizeOplus();

  // compare the Jacobians
  number_t* n = numericJacobianWorkspace.workspaceForVertex(0);
  number_t* a = jacobianWorkspace.workspaceForVertex(0);
  int numElems = EdgeType::Dimension;
  numElems *= EdgeType::VertexXiType::Dimension;
  for (int j = 0; j < numElems; ++j) {
    EXPECT_NEAR(n[j], a[j], 1e-6);
  }
}

template <typename EdgeType>
void evaluateJacobian(EdgeType& e, JacobianWorkspace& jacobianWorkspace, JacobianWorkspace& numericJacobianWorkspace)
{
    // calling the analytic Jacobian but writing to the numeric workspace
    e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
      typename EdgeType::VertexXiType, typename EdgeType::VertexXjType>::linearizeOplus(numericJacobianWorkspace);
    // copy result into analytic workspace
    jacobianWorkspace = numericJacobianWorkspace;

    // compute the numeric Jacobian into the numericJacobianWorkspace workspace as setup by the previous call
    e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
      typename EdgeType::VertexXiType, typename EdgeType::VertexXjType>::linearizeOplus();

    // compare the two Jacobians
    for (int i = 0; i < 2; ++i) {
      number_t* n = numericJacobianWorkspace.workspaceForVertex(i);
      number_t* a = jacobianWorkspace.workspaceForVertex(i);
      int numElems = EdgeType::Dimension;
      if (i == 0)
        numElems *= EdgeType::VertexXiType::Dimension;
      else
        numElems *= EdgeType::VertexXjType::Dimension;
      for (int j = 0; j < numElems; ++j) {
        EXPECT_NEAR(n[j], a[j], 1e-6);
      }
    }
}

} // end namespace

#endif
