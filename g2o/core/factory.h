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

#ifndef G2O_FACTORY_H
#define G2O_FACTORY_H

#include "hyper_graph.h"
#include "creators.h"

#include <string>
#include <map>
#include <iostream>

namespace g2o {

  class AbstractHyperGraphElementCreator;
  
  /**
   * \brief create vertices and edges based on TAGs in, for example, a file
   */
  class G2O_CORE_API Factory
  {
    public:

      //! return the instance
      static Factory* instance();

      //! free the instance
      static void destroy();

      /**
       * register a tag for a specific creator
       */
      void registerType(const std::string& tag, AbstractHyperGraphElementCreator* c);

      /**
       * unregister a tag for a specific creator
       */
      void unregisterType(const std::string& tag);

      /**
       * construct a graph element based on its tag
       */
      HyperGraph::HyperGraphElement* construct(const std::string& tag) const;

      /**
       * construct a graph element based on its tag, but only if it's type (a bitmask) matches. A bitmask without any
       * bit set will construct any item. Otherwise a bit has to be set to allow construction of a graph element.
       */
      HyperGraph::HyperGraphElement* construct(const std::string& tag, const HyperGraph::GraphElemBitset& elemsToConstruct) const;

      /**
       * return whether the factory knows this tag or not
       */
      bool knowsTag(const std::string& tag, int* elementType = 0) const;

      //! return the TAG given a vertex
      const std::string& tag(const HyperGraph::HyperGraphElement* v) const;

      /**
       * get a list of all known types
       */
      void fillKnownTypes(std::vector<std::string>& types) const;

      /**
       * print a list of the known registered types to the given stream
       */
      void printRegisteredTypes(std::ostream& os, bool comment = false) const;

    protected:
      class CreatorInformation
      {
        public:
          AbstractHyperGraphElementCreator* creator;
          int elementTypeBit;
          CreatorInformation()
          {
            creator = 0;
            elementTypeBit = -1;
          }
        
          ~CreatorInformation()
          {
            std::cout << "Deleting " << (void*) creator << std::endl;
            
            delete creator;
          }
      };

      typedef std::map<std::string, CreatorInformation*>               CreatorMap;
      typedef std::map<std::string, std::string>                      TagLookup;
      Factory();
      ~Factory();

      CreatorMap _creator;     ///< look-up map for the existing creators
      TagLookup _tagLookup;    ///< reverse look-up, class name to tag

    private:
      static Factory* factoryInstance;
  };

  template<typename T> class RegisterTypeProxy
    {
      public:
      RegisterTypeProxy(const std::string& name) : _name(name)
          {
#ifndef NDEBUG
            std::cout << __FUNCTION__ << ": Registering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
            Factory::instance()->registerType(_name, new HyperGraphElementCreator<T>());
          }
      
        ~RegisterTypeProxy()
          {
#ifndef NDEBUG
            std::cout << __FUNCTION__ << ": Unregistering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
            Factory::instance()->unregisterType(_name);
          }

    private:
      std::string _name;
  };

#define REGISTER_G2O_TYPE(name, classname) \
    extern "C" void g2o_type_##classname(void) {} \
    static g2o::RegisterTypeProxy<classname> g_proxy_##classname(#name);
}

#endif
