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

#include "optimization_algorithm.h"

using namespace std;

namespace g2o {

OptimizationAlgorithm::OptimizationAlgorithm() :
  _optimizer(0)
{
}

OptimizationAlgorithm::~OptimizationAlgorithm()
{
}

void OptimizationAlgorithm::printProperties(std::ostream& os) const
{
  os << "------------- Algorithm Properties -------------"  << endl;
  for (PropertyMap::const_iterator it = _properties.begin(); it != _properties.end(); ++it) {
    BaseProperty* p = it->second;
    os << it->first << "\t" << p->toString() << endl;
  }
  os << "------------------------------------------------" << endl;
}

bool OptimizationAlgorithm::updatePropertiesFromString(const std::string& propString)
{
  return _properties.updateMapFromString(propString);
}

void OptimizationAlgorithm::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

} // end namespace
