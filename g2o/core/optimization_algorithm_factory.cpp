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

#include "optimization_algorithm_factory.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <utility>

#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/stuff/logger.h"

namespace g2o {

AbstractOptimizationAlgorithmCreator::AbstractOptimizationAlgorithmCreator(
    OptimizationAlgorithmProperty p)
    : property_(std::move(p)) {}

std::unique_ptr<OptimizationAlgorithmFactory>
    OptimizationAlgorithmFactory::factoryInstance_;

OptimizationAlgorithmFactory* OptimizationAlgorithmFactory::instance() {
  if (factoryInstance_ == nullptr) {
    factoryInstance_.reset(new OptimizationAlgorithmFactory);
  }
  return factoryInstance_.get();
}

void OptimizationAlgorithmFactory::registerSolver(
    const std::shared_ptr<AbstractOptimizationAlgorithmCreator>& c) {
  const std::string& name = c->property().name;
  auto foundIt = findSolver(name);
  if (foundIt != creator_.end()) {
    creator_.erase(foundIt);
    G2O_WARN("SOLVER FACTORY: Overwriting Solver creator {}", name);
    assert(0);
  }
  creator_.push_back(c);
}

void OptimizationAlgorithmFactory::unregisterSolver(
    const std::shared_ptr<AbstractOptimizationAlgorithmCreator>& c) {
  const std::string& name = c->property().name;
  auto foundIt = findSolver(name);
  if (foundIt != creator_.end()) {
    creator_.erase(foundIt);
  }
}

std::unique_ptr<OptimizationAlgorithm> OptimizationAlgorithmFactory::construct(
    const std::string& name,
    OptimizationAlgorithmProperty& solverProperty) const {
  auto foundIt = findSolver(name);
  if (foundIt != creator_.end()) {
    solverProperty = (*foundIt)->property();
    return (*foundIt)->construct();
  }
  G2O_WARN("SOLVER FACTORY: Unable to create solver {}", name);
  return std::unique_ptr<OptimizationAlgorithm>();
}

void OptimizationAlgorithmFactory::destroy() {
  std::unique_ptr<OptimizationAlgorithmFactory> aux;
  factoryInstance_.swap(aux);
}

void OptimizationAlgorithmFactory::listSolvers(std::ostream& os) const {
  size_t solverNameColumnLength = 0;
  for (const auto& it : creator_)
    solverNameColumnLength =
        std::max(solverNameColumnLength, it->property().name.size());
  solverNameColumnLength += 4;

  for (const auto& it : creator_) {
    const OptimizationAlgorithmProperty& sp = it->property();
    os << sp.name;
    for (size_t i = sp.name.size(); i < solverNameColumnLength; ++i) os << ' ';
    os << sp.type << " \t" << sp.desc << '\n';
  }
}

OptimizationAlgorithmFactory::CreatorList::const_iterator
OptimizationAlgorithmFactory::findSolver(const std::string& name) const {
  for (auto it = creator_.begin(); it != creator_.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (sp.name == name) return it;
  }
  return creator_.end();
}

OptimizationAlgorithmFactory::CreatorList::iterator
OptimizationAlgorithmFactory::findSolver(const std::string& name) {
  for (auto it = creator_.begin(); it != creator_.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (sp.name == name) return it;
  }
  return creator_.end();
}

RegisterOptimizationAlgorithmProxy::RegisterOptimizationAlgorithmProxy(
    std::shared_ptr<AbstractOptimizationAlgorithmCreator> c)
    : creator_(std::move(c)) {
  const AbstractOptimizationAlgorithmCreator* ptr = creator_.get();
  (void)ptr;
  G2O_DEBUG("Registering optimization algorithm {} of type {}",
            creator_->property().name, typeid(*ptr).name());
  OptimizationAlgorithmFactory::instance()->registerSolver(creator_);
}

}  // namespace g2o
