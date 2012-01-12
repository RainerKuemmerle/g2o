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

#ifndef G2O_CREATORS_H
#define G2O_CREATORS_H

#include "hyper_graph.h"

#include <string>
#include <typeinfo>

namespace g2o
{

  /**
   * \brief Abstract interface for allocating HyperGraphElement
   */
  class AbstractHyperGraphElementCreator
  {
    public:
      /**
       * create a hyper graph element. Has to implemented in derived class.
       */
      virtual HyperGraph::HyperGraphElement* construct() = 0;
      /**
       * name of the class to be created. Has to implemented in derived class.
       */
      virtual const std::string& name() const = 0;
  };

  /**
   * \brief templatized creator class which creates graph elements
   */
  template <typename T>
  class HyperGraphElementCreator : public AbstractHyperGraphElementCreator
  {
    public:
      HyperGraphElementCreator() : _name(typeid(T).name()) {}
      HyperGraph::HyperGraphElement* construct() { return new T;}
      const std::string& name() const { return _name;}
    protected:
      std::string _name;
  };

} // end namespace

#endif
