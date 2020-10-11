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

#ifndef TESTHELPER_IO_H
#define TESTHELPER_IO_H

#include <Eigen/Core>
#include <sstream>

#include "gtest/gtest.h"

namespace g2o {

template <typename T>
void readWriteGraphElement(const T& output, T* input) {
  std::stringstream data;
  EXPECT_TRUE(output.write(data));
  EXPECT_TRUE(input->read(data));
}

template <typename Derived>
void randomizeInformationMatrix(Eigen::MatrixBase<Derived>& m) {
  m = Derived::Random(m.rows(), m.cols());
  m = m * m.transpose();
  m += m.rows() * Derived::Identity();
}

/**
 * Small wrapper to allow passing optional pointers
 * TODO when we switch to C++17 use std::optional instead
 */
template <typename T>
struct OptionalPtr {
    OptionalPtr() : ptr(new T), own(true) {}
    OptionalPtr(T* p) : ptr(p), own(false) {}
    T* ptr;
    bool own;
};

template <typename T>
struct RandomEstimate {
  static typename T::EstimateType create() { return T::EstimateType::Random(); }
  static bool isApprox(const typename T::EstimateType& a, const typename T::EstimateType& b) { return a.isApprox(b, 1e-5); }
};

template <typename T>
struct RandomMeasurement {
  static typename T::Measurement create() { return T::Measurement::Random(); }
  static bool isApprox(const typename T::Measurement& a, const typename T::Measurement& b) { return a.isApprox(b, 1e-5); }
};

template <typename T, typename RandomEstimateFunctor = RandomEstimate<T>>
void readWriteVectorBasedVertex(OptionalPtr<T> outputVertex = OptionalPtr<T>()) {
  outputVertex.ptr->setEstimate(RandomEstimateFunctor::create());
  T inputVertex;
  readWriteGraphElement(*outputVertex.ptr, &inputVertex);
  ASSERT_TRUE(RandomEstimateFunctor::isApprox(outputVertex.ptr->estimate(), inputVertex.estimate()));
  if (outputVertex.own) delete outputVertex.ptr;
}

template <typename T, typename RandomMeasurementFunctor = RandomMeasurement<T>>
void readWriteVectorBasedEdge(OptionalPtr<T> outputEdge = OptionalPtr<T>()) {
  outputEdge.ptr->setMeasurement(RandomMeasurementFunctor::create());
  randomizeInformationMatrix(outputEdge.ptr->information());
  T inputEdge;
  readWriteGraphElement(*outputEdge.ptr, &inputEdge);
  ASSERT_TRUE(RandomMeasurementFunctor::isApprox(outputEdge.ptr->measurement(), inputEdge.measurement()));
  ASSERT_TRUE(outputEdge.ptr->information().isApprox(inputEdge.information(), 1e-5));
  if (outputEdge.own) delete outputEdge.ptr;
}

}  // namespace g2o

#endif
