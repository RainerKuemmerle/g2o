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

#include "allocate_optimizer.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/stuff/misc.h"
#include "gmock/gmock.h"

using namespace testing;

TEST(General, RobustKernelFactory) {
  g2o::RobustKernelFactory* factory = g2o::RobustKernelFactory::instance();

  std::vector<std::string> kernels;
  factory->fillKnownKernels(kernels);

  ASSERT_THAT(kernels, SizeIs(Gt(0)));

  for (const auto& s : kernels) {
    ASSERT_THAT(factory->creator(s), NotNull());
  }
  for (const auto& s : kernels) {
    g2o::RobustKernel* kernel = factory->construct(s);
    ASSERT_THAT(kernel, NotNull());
    delete kernel;
  }

  // remove one kernel
  std::string kernelToRemove = kernels[kernels.size() / 2];
  factory->unregisterType(kernelToRemove);
  // check that we cannot create it
  ASSERT_THAT(factory->creator(kernelToRemove), IsNull());
  ASSERT_THAT(factory->construct(kernelToRemove), IsNull());
  // check that we removed exactky that desired kernel
  std::vector<std::string> kernelsAfterRemoval;
  factory->fillKnownKernels(kernelsAfterRemoval);
  ASSERT_THAT(kernelsAfterRemoval, Not(Contains(kernelToRemove)));
  ASSERT_THAT(kernelsAfterRemoval, IsSubsetOf(kernels));
  ASSERT_THAT(kernelsAfterRemoval, SizeIs(kernels.size() - 1));
  kernelsAfterRemoval.push_back(kernelToRemove);
  ASSERT_THAT(kernelsAfterRemoval, UnorderedElementsAreArray(kernels));
}

/**
 * Type parameterized class for a fixture to setup a linear solver along with
 * some data of a linear system to be solved.
 */
template <typename T>
class RobustKernelTests : public Test {
 public:
  using Kernel = T;

  RobustKernelTests() : Test() {
    kernel_.setDelta(0.07);

    error_values_.reserve(3);
    error_values_.push_back(0.99 * kernel_.delta() * kernel_.delta());
    error_values_.push_back(0.99 * kernel_.delta());
    error_values_.push_back(1.5 * kernel_.delta());
  }

 protected:
  Kernel kernel_;
  std::vector<double> error_values_;

  //! Evaluate the robust kernel function
  double eval(double val) const {
    g2o::Vector3 result;
    kernel_.robustify(val, result);
    return result[0];
  }

  /**
   * Estimate the first order derivative numerically
   */
  double estimateDerivative(double x) {
    constexpr double delta = g2o::cst(1e-9);

    double result = (1 / (2 * delta)) * (eval(x + delta) - eval(x - delta));
    return result;
  }

  double estimate2ndDerivative(double x) {
    constexpr double kEps = 1e-5;

    const double result1 =
        (1. / (kEps * kEps)) * (2. * eval(x) - 5. * eval(x - kEps) +
                                4 * eval(x - 2. * kEps) - eval(x - 3 * kEps));
    const double result2 =
        (1. / (kEps * kEps)) * (2. * eval(x) - 5. * eval(x + kEps) +
                                4 * eval(x + 2. * kEps) - eval(x + 3 * kEps));
    return 0.5 * result1 + 0.5 * result2;
  }
};

TYPED_TEST_SUITE_P(RobustKernelTests);

TYPED_TEST_P(RobustKernelTests, Values) {
  for (auto e : this->error_values_) {
    g2o::Vector3 val = g2o::Vector3::Zero();
    this->kernel_.robustify(e, val);
    EXPECT_THAT(val(0), Ne(0.));
  }
}

TYPED_TEST_P(RobustKernelTests, Derivative) {
  for (auto e : this->error_values_) {
    g2o::Vector3 val = g2o::Vector3::Zero();
    this->kernel_.robustify(e, val);
    double estimatedJac = this->estimateDerivative(e);
    EXPECT_THAT(val(1), DoubleNear(estimatedJac, 1e-5));
  }
}

TYPED_TEST_P(RobustKernelTests, SecondDerivative) {
  for (auto e : this->error_values_) {
    g2o::Vector3 val = g2o::Vector3::Zero();
    this->kernel_.robustify(e, val);
    double estimated2ndDerivative = this->estimate2ndDerivative(e);
    EXPECT_THAT(val(2), DoubleNear(estimated2ndDerivative, 1e-3));
  }
}

// clang-format off
REGISTER_TYPED_TEST_SUITE_P(RobustKernelTests, Values, Derivative, SecondDerivative);
using RobustKernelTypes = ::testing::Types<
  g2o::RobustKernelHuber,
  g2o::RobustKernelPseudoHuber,
  g2o::RobustKernelCauchy,
  g2o::RobustKernelGemanMcClure,
  g2o::RobustKernelWelsch,
  g2o::RobustKernelFair,
  g2o::RobustKernelTukey,
  g2o::RobustKernelSaturated,
  g2o::RobustKernelDCS
>;
INSTANTIATE_TYPED_TEST_SUITE_P(General, RobustKernelTests, RobustKernelTypes);
// clang-format on
