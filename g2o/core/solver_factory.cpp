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

#include "solver_factory.h"

#include <iostream>
#include <typeinfo>
#include <cassert>

using namespace std;

namespace g2o {

  AbstractSolverCreator::AbstractSolverCreator(const SolverProperty& p) :
    _property(p)
  {
  }

  SolverFactory* SolverFactory::factoryInstance = 0;

  SolverFactory::SolverFactory()
  {
  }

  SolverFactory::~SolverFactory()
  {
    for (CreatorList::iterator it = _creator.begin(); it != _creator.end(); ++it)
      delete *it;
  }

  SolverFactory* SolverFactory::instance()
  {
    if (factoryInstance == 0) {
      factoryInstance = new SolverFactory;
    }
    return factoryInstance;
  }

  void SolverFactory::registerSolver(AbstractSolverCreator* c)
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

  OptimizationAlgorithm* SolverFactory::construct(const std::string& name, SolverProperty& solverProperty) const
  {
    CreatorList::const_iterator foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      solverProperty = (*foundIt)->property();
      return (*foundIt)->construct();
    }
    cerr << "SOLVER FACTORY WARNING: Unable to create solver " << name << endl;
    return 0;
  }

  void SolverFactory::destroy()
  {
    delete factoryInstance;
    factoryInstance = 0;
  }

  void SolverFactory::listSolvers(std::ostream& os) const
  {
    for (CreatorList::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const SolverProperty& sp = (*it)->property();
      os << sp.name << "\t " << sp.desc << endl;
    }
  }

  SolverFactory::CreatorList::const_iterator SolverFactory::findSolver(const std::string& name) const
  {
    for (CreatorList::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const SolverProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

  SolverFactory::CreatorList::iterator SolverFactory::findSolver(const std::string& name)
  {
    for (CreatorList::iterator it = _creator.begin(); it != _creator.end(); ++it) {
      const SolverProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

} // end namespace
