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

// clang-format off
#include "g2o/config.h"
// clang-format on

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm.h"
#include "gtest/gtest.h"

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#endif
#ifdef G2O_HAVE_CSPARSE
G2O_USE_OPTIMIZATION_LIBRARY(csparse);
#endif

G2O_USE_OPTIMIZATION_LIBRARY(eigen);
G2O_USE_OPTIMIZATION_LIBRARY(dense);
G2O_USE_OPTIMIZATION_LIBRARY(pcg);
G2O_USE_OPTIMIZATION_LIBRARY(structure_only);
G2O_USE_OPTIMIZATION_LIBRARY(slam2d_linear);

TEST(AlgorithmFactory, ContainsBasicSolvers) {
  g2o::OptimizationAlgorithmFactory* factory = g2o::OptimizationAlgorithmFactory::instance();

  // collect all names
  std::set<std::string> names;
  for (auto& creator : factory->creatorList()) {
    g2o::OptimizationAlgorithmProperty solverProperty;
    names.insert(creator->property().name);
  }

  std::vector<std::string> basicSolvers = {"gn_var", "lm_var", "dl_var"};
  for (const auto& name : basicSolvers) {
    ASSERT_EQ(1, names.count(name))
        << "solver with name \"" << name << "\" not in the set of known solvers";
  }
}

TEST(AlgorithmFactory, AllocatingSolver) {
  g2o::OptimizationAlgorithmFactory* factory = g2o::OptimizationAlgorithmFactory::instance();

  for (auto& creator : factory->creatorList()) {
    g2o::OptimizationAlgorithmProperty solverProperty;
    std::string name = creator->property().name;

    g2o::OptimizationAlgorithm* algo = factory->construct(name, solverProperty);
    ASSERT_NE(nullptr, algo) << "Cannot allocate solver " << name;
    delete algo;
  }
}
