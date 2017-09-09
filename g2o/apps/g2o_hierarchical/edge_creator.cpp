// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "g2o/core/factory.h"
#include "edge_creator.h"

namespace g2o {

  using namespace std;

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType, const std::vector<int>& parameterIds) {

    EntryMap::iterator it = _vertexToEdgeMap.find(vertexTypes);
    if (it!=_vertexToEdgeMap.end())
      it->second = edgeType;
    else
      _vertexToEdgeMap.insert(make_pair(vertexTypes,EdgeCreatorEntry(edgeType, parameterIds)));
    return true;
  }

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType) {
    return addAssociation(vertexTypes, edgeType, std::vector<int>());
  }

  bool EdgeCreator::removeAssociation(std::string vertexTypes){
    EntryMap::iterator it = _vertexToEdgeMap.find(vertexTypes);
    if (it==_vertexToEdgeMap.end())
      return false;
    _vertexToEdgeMap.erase(it);
    return true;
  }


  OptimizableGraph::Edge* EdgeCreator::createEdge(std::vector<OptimizableGraph::Vertex*>& vertices ){
    std::stringstream key;
    Factory* factory=Factory::instance();
    for (size_t i=0; i<vertices.size(); i++){
      key << factory->tag(vertices[i]) << ";";
    }
    EntryMap::iterator it=_vertexToEdgeMap.find(key.str());
    if (it==_vertexToEdgeMap.end()){
      cerr << "no thing in factory: " << key.str() << endl;
      return 0;
    }
    HyperGraph::HyperGraphElement* element=factory->construct(it->second._edgeTypeName);
    if (! element) {
      cerr << "no thing can be created" << endl;
      return 0;
    }
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(element);
    assert(it->second._parameterIds.size() == e->numParameters());
    for (size_t i=0; i<it->second._parameterIds.size(); i++){
      if (! e->setParameterId(i,it->second._parameterIds[i])) {
        cerr << "no thing in good for setting params" << endl;
        return 0;
      }
    }
    assert (e);
    for (size_t i=0; i<vertices.size(); i++)
      e->vertices()[i]=vertices[i];
    return e;
  }

}

