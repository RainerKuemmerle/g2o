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

#ifndef G2O_CACHE_HH_
#define G2O_CACHE_HH_

#include <map>

#include "optimizable_graph.h"

namespace g2o {

  class CacheContainer;
  
  class Cache: public HyperGraph::HyperGraphElement
  {
    public:
      friend class CacheContainer;
      class CacheKey
      {
        public:
          friend class CacheContainer;
          CacheKey(const std::string& type_, const ParameterVector& parameters_);

          bool operator<(const CacheKey& c) const;

          const std::string& type() const { return _type;}
          const ParameterVector& parameters() const { return _parameters;}

        protected:
          std::string _type;
          ParameterVector _parameters;
      };

      Cache(CacheContainer* container_ = 0, const ParameterVector& parameters_ = ParameterVector());

      CacheKey key() const;

      OptimizableGraph::Vertex* vertex();
      OptimizableGraph* graph();
      CacheContainer* container();
      ParameterVector& parameters();

      void update();

      virtual HyperGraph::HyperGraphElementType elementType() const { return HyperGraph::HGET_CACHE;}

    protected:
      //! redefine this to do the update
      virtual void updateImpl() = 0;

      /**
       * this function installs and satisfies a cache
       * @param type_: the typename of the dependancy
       * @param parameterIndices: a vector containing the indices if the parameters
       * in _parameters that will be used to assemble the Key of the cache being created
       * For example if I have a cache of type C2, having parameters "A, B, and C",
       * and it depends on a cache of type C1 that depends on the parameters A and C, 
       * the parameterIndices should contain "0, 2", since they are the positions in the
       * parameter vector of C2 of the parameters needed to construct C1.
       * @returns the newly created cache
       */
      Cache* installDependancy(const std::string& type_, const std::vector<int>& parameterIndices);

      /**
       * Function to be called from a cache that has dependencies. It just invokes a
       * sequence of installDependancy().
       * Although the caches returned are stored in the _parentCache vector,
       * it is better that you redefine your own cache member variables, for better readability
       */
      virtual bool resolveDependancies();

      bool _updateNeeded;
      ParameterVector _parameters;
      std::vector<Cache*> _parentCaches;
      CacheContainer* _container;
  };

  class CacheContainer: public std::map<Cache::CacheKey, Cache*>
  {
    public:
      CacheContainer(OptimizableGraph::Vertex* vertex_);
      virtual ~CacheContainer();
      OptimizableGraph::Vertex* vertex();
      OptimizableGraph* graph();
      Cache* findCache(const Cache::CacheKey& key);
      Cache* createCache(const Cache::CacheKey& key);
      void setUpdateNeeded(bool needUpdate=true);
      void update();
    protected:
      OptimizableGraph::Vertex* _vertex;
      bool _updateNeeded;
  };


  template <typename CacheType>
  void OptimizableGraph::Edge::resolveCache(CacheType*& cache, 
      OptimizableGraph::Vertex* v, 
      const std::string& type_, 
      const ParameterVector& parameters_)
  {
    cache = 0;
    CacheContainer* container= v->cacheContainer();
    Cache::CacheKey key(type_, parameters_);
    Cache* c = container->findCache(key);
    if (!c) {
      c = container->createCache(key);
    }
    if (c) {
      cache = dynamic_cast<CacheType*>(c); 
    }
  }

} // end namespace

#endif
