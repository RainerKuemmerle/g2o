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

#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/stuff/misc.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/allocate_optimizer.h"

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
    auto kernel = factory->construct(s);
    ASSERT_THAT(kernel, NotNull());
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
    error_values_.push_back(0.5 * kernel_.delta());
    error_values_.push_back(0.99 * kernel_.delta());
    error_values_.push_back(1.5 * kernel_.delta());
  }

 protected:
  Kernel kernel_;
  std::vector<double> error_values_;

  /**
   * Estimate the first order derivative numerically
   */
  number_t estimateDerivative(number_t x) {
    constexpr number_t delta = g2o::cst(1e-9);

    g2o::Vector3 first;
    g2o::Vector3 second;
    this->kernel_.robustify(x + delta, first);
    this->kernel_.robustify(x - delta, second);

    number_t result = (1 / (2 * delta)) * (first(0) - second(0));
    return result;
  }
};
TYPED_TEST_SUITE_P(RobustKernelTests);

TYPED_TEST_P(RobustKernelTests, Values) {
  for (auto e : this->error_values_) {
    g2o::Vector3 val = g2o::Vector3::Zero();
    this->kernel_.robustify(e, val);
    ASSERT_THAT(val(0), Not(Eq(0.)));
  }
}

TYPED_TEST_P(RobustKernelTests, Derivative) {
  for (auto e : this->error_values_) {
    g2o::Vector3 val = g2o::Vector3::Zero();
    this->kernel_.robustify(e, val);
    number_t estimatedJac = this->estimateDerivative(e);
    ASSERT_THAT(val(1), DoubleNear(estimatedJac, 1e-5));
  }
}

// clang-format off
REGISTER_TYPED_TEST_SUITE_P(RobustKernelTests, Values, Derivative);
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
