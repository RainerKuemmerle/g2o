// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "optimization_algorithm_factory.h"

#include <iostream>
#include <typeinfo>
#include <cassert>

using namespace std;

namespace g2o {

  AbstractOptimizationAlgorithmCreator::AbstractOptimizationAlgorithmCreator(const OptimizationAlgorithmProperty& p) :
    _property(p)
  {
  }

  OptimizationAlgorithmFactory* OptimizationAlgorithmFactory::factoryInstance = 0;

  OptimizationAlgorithmFactory::OptimizationAlgorithmFactory()
  {
  }

  OptimizationAlgorithmFactory::~OptimizationAlgorithmFactory()
  {
    for (CreatorList::iterator it = _creator.begin(); it != _creator.end(); ++it)
      delete *it;
  }

  OptimizationAlgorithmFactory* OptimizationAlgorithmFactory::instance()
  {
    if (factoryInstance == 0) {
      factoryInstance = new OptimizationAlgorithmFactory;
    }
    return factoryInstance;
  }

  void OptimizationAlgorithmFactory::registerSolver(AbstractOptimizationAlgorithmCreator* c)
  {
    const string& name = c->property().name;
    CreatorList::iterator foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      _creator.erase(foundIt);
      cerr << "SOLVER FACTORY WARNING: Overwriting Solver creator " << name << endl;
      assert(0);
    }
    _creator.push_back(c);
  }

  OptimizationAlgorithm* OptimizationAlgorithmFactory::construct(const std::string& name, OptimizationAlgorithmProperty& solverProperty) const
  {
    CreatorList::const_iterator foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      solverProperty = (*foundIt)->property();
      return (*foundIt)->construct();
    }
    cerr << "SOLVER FACTORY WARNING: Unable to create solver " << name << endl;
    return 0;
  }

  void OptimizationAlgorithmFactory::destroy()
  {
    delete factoryInstance;
    factoryInstance = 0;
  }

  void OptimizationAlgorithmFactory::listSolvers(std::ostream& os) const
  {
    for (CreatorList::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      os << sp.name << "\t " << sp.desc << endl;
    }
  }

  OptimizationAlgorithmFactory::CreatorList::const_iterator OptimizationAlgorithmFactory::findSolver(const std::string& name) const
  {
    for (CreatorList::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

  OptimizationAlgorithmFactory::CreatorList::iterator OptimizationAlgorithmFactory::findSolver(const std::string& name)
  {
    for (CreatorList::iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

} // end namespace
