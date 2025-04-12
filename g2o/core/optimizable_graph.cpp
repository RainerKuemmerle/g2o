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

#include "optimizable_graph.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#include "cache.h"
#include "factory.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/logger_format.h"  // IWYU pragma: keep
#include "g2o/stuff/string_tools.h"
#include "hyper_graph_action.h"
#include "optimization_algorithm_property.h"
#include "ownership.h"
#include "robust_kernel.h"

namespace g2o {

using namespace std;

OptimizableGraph::Vertex::Vertex()
    : HyperGraph::Vertex(),
      _graph(0),
      _userData(0),
      _hessianIndex(-1),
      _fixed(false),
      _marginalized(false),
      _colInHessian(-1),
      _cacheContainer(0) {}

CacheContainer* OptimizableGraph::Vertex::cacheContainer() {
  if (!_cacheContainer) _cacheContainer = new CacheContainer(this);
  return _cacheContainer;
}

void OptimizableGraph::Vertex::updateCache() {
  if (_cacheContainer) {
    _cacheContainer->setUpdateNeeded();
    _cacheContainer->update();
  }
}

OptimizableGraph::Vertex::~Vertex() {
  delete _cacheContainer;
  delete _userData;
}

bool OptimizableGraph::Vertex::setEstimateData(const double* v) {
  bool ret = setEstimateDataImpl(v);
  updateCache();
  return ret;
}

bool OptimizableGraph::Vertex::getEstimateData(double*) const { return false; }

int OptimizableGraph::Vertex::estimateDimension() const { return -1; }

bool OptimizableGraph::Vertex::setMinimalEstimateData(const double* v) {
  bool ret = setMinimalEstimateDataImpl(v);
  updateCache();
  return ret;
}

bool OptimizableGraph::Vertex::getMinimalEstimateData(double*) const {
  return false;
}

int OptimizableGraph::Vertex::minimalEstimateDimension() const { return -1; }

OptimizableGraph::Edge::Edge()
    : HyperGraph::Edge(), _dimension(-1), _level(0), _robustKernel(nullptr) {}

OptimizableGraph::Edge::~Edge() { release(_robustKernel); }

OptimizableGraph* OptimizableGraph::Edge::graph() {
  if (!_vertices.size()) return nullptr;
  OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)_vertices[0];
  if (!v) return nullptr;
  return v->graph();
}

const OptimizableGraph* OptimizableGraph::Edge::graph() const {
  if (!_vertices.size()) return nullptr;
  const OptimizableGraph::Vertex* v =
      (const OptimizableGraph::Vertex*)_vertices[0];
  if (!v) return nullptr;
  return v->graph();
}

bool OptimizableGraph::Edge::setParameterId(int argNum, int paramId) {
  if ((int)_parameters.size() <= argNum) return false;
  if (argNum < 0) return false;
  *_parameters[argNum] = 0;
  _parameterIds[argNum] = paramId;
  return true;
}

bool OptimizableGraph::Edge::resolveParameters() {
  if (!graph()) {
    G2O_ERROR("edge not registered with a graph");
    return false;
  }

  assert(_parameters.size() == _parameterIds.size());
  for (size_t i = 0; i < _parameters.size(); i++) {
    int index = _parameterIds[i];
    *_parameters[i] = graph()->parameter(index);
    if (!*_parameters[i]) {
      G2O_CRITICAL(
          "parameter {} is NULL. Check setParameterId() calls for the edge.",
          i);
      return false;
    }
    auto& aux = **_parameters[i];
    if (typeid(aux).name() != _parameterTypes[i]) {
      G2O_CRITICAL("parameter type mismatch - encountered {}; should be {}",
                   typeid(aux).name(), _parameterTypes[i]);
    }
  }
  return true;
}

void OptimizableGraph::Edge::setRobustKernel(RobustKernel* ptr) {
  if (_robustKernel) release(_robustKernel);

  _robustKernel = ptr;
}

bool OptimizableGraph::Edge::resolveCaches() { return true; }

bool OptimizableGraph::Edge::setMeasurementData(const double*) { return false; }

bool OptimizableGraph::Edge::getMeasurementData(double*) const { return false; }

int OptimizableGraph::Edge::measurementDimension() const { return -1; }

bool OptimizableGraph::Edge::setMeasurementFromState() { return false; }

OptimizableGraph::OptimizableGraph() {
  _nextEdgeId = 0;
  _graphActions.resize(AT_NUM_ELEMENTS);
}

OptimizableGraph::~OptimizableGraph() {
  clear();
  clearParameters();
}

bool OptimizableGraph::addVertex(OptimizableGraph::Vertex* ov, Data* userData) {
  if (ov->id() < 0) {
    G2O_ERROR(
        "FATAL, a vertex with (negative) ID {} cannot be inserted into the "
        "graph",
        ov->id());
    assert(0 && "Invalid vertex id");
    return false;
  }
  Vertex* inserted = vertex(ov->id());
  if (inserted) {
    G2O_WARN(
        "a vertex with ID {} has already been registered with this "
        "graph",
        ov->id());
    return false;
  }
  if (ov->_graph != nullptr && ov->_graph != this) {
    G2O_ERROR(
        "FATAL, vertex with ID {} has already been registered with another "
        "graph {}",
        ov->id(), static_cast<void*>(ov->_graph));
    return false;
  }
  if (userData) ov->setUserData(userData);
  ov->_graph = this;
  return HyperGraph::addVertex(ov);
}

bool OptimizableGraph::addVertex(HyperGraph::Vertex* v, Data* userData) {
  OptimizableGraph::Vertex* ov = dynamic_cast<OptimizableGraph::Vertex*>(v);
  assert(ov && "Vertex does not inherit from OptimizableGraph::Vertex");
  if (!ov) return false;

  return addVertex(ov, userData);
}

bool OptimizableGraph::addEdge(OptimizableGraph::Edge* e) {
  OptimizableGraph* g = e->graph();

  if (g != nullptr && g != this) {
    G2O_ERROR(
        "FATAL, edge with ID {} has already registered with another graph "
        "{}",
        e->id(), static_cast<void*>(g));
    return false;
  }

  bool eresult = HyperGraph::addEdge(e);
  if (!eresult) return false;

  e->_internalId = _nextEdgeId++;
  if (e->numUndefinedVertices()) return true;
  if (!e->resolveParameters()) {
    G2O_ERROR("FATAL, cannot resolve parameters for edge {}",
              static_cast<void*>(e));
    return false;
  }
  if (!e->resolveCaches()) {
    G2O_ERROR("FATAL, cannot resolve caches for edge {}",
              static_cast<void*>(e));
    return false;
  }

  _jacobianWorkspace.updateSize(e);

  return true;
}

bool OptimizableGraph::addEdge(HyperGraph::Edge* e_) {
  OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(e_);
  assert(e && "Edge does not inherit from OptimizableGraph::Edge");
  if (!e) return false;
  return addEdge(e);
}

bool OptimizableGraph::setEdgeVertex(HyperGraph::Edge* e, int pos,
                                     HyperGraph::Vertex* v) {
  if (!HyperGraph::setEdgeVertex(e, pos, v)) {
    return false;
  }
  if (!e->numUndefinedVertices()) {
#ifndef NDEBUG
    OptimizableGraph::Edge* ee = dynamic_cast<OptimizableGraph::Edge*>(e);
    assert(ee && "Edge is not a OptimizableGraph::Edge");
#else
    OptimizableGraph::Edge* ee = static_cast<OptimizableGraph::Edge*>(e);
#endif
    if (!ee->resolveParameters()) {
      G2O_ERROR("FATAL, cannot resolve parameters for edge {}",
                static_cast<void*>(e));
      return false;
    }
    if (!ee->resolveCaches()) {
      G2O_ERROR("FATAL, cannot resolve caches for edge {}",
                static_cast<void*>(e));
      return false;
    }
    _jacobianWorkspace.updateSize(e);
  }
  return true;
}

int OptimizableGraph::optimize(int /*iterations*/, bool /*online*/) {
  return 0;
}

double OptimizableGraph::chi2() const {
  double chi = 0.0;
  for (OptimizableGraph::EdgeSet::const_iterator it = this->edges().begin();
       it != this->edges().end(); ++it) {
    const OptimizableGraph::Edge* e =
        static_cast<const OptimizableGraph::Edge*>(*it);
    chi += e->chi2();
  }
  return chi;
}

void OptimizableGraph::push() {
  forEachVertex([](OptimizableGraph::Vertex* v) { v->push(); });
}

void OptimizableGraph::pop() {
  forEachVertex([](OptimizableGraph::Vertex* v) { v->pop(); });
}

void OptimizableGraph::discardTop() {
  forEachVertex([](OptimizableGraph::Vertex* v) { v->discardTop(); });
}

void OptimizableGraph::push(HyperGraph::VertexSet& vset) {
  forEachVertex(vset, [](OptimizableGraph::Vertex* v) { v->push(); });
}

void OptimizableGraph::pop(HyperGraph::VertexSet& vset) {
  forEachVertex(vset, [](OptimizableGraph::Vertex* v) { v->pop(); });
}

void OptimizableGraph::discardTop(HyperGraph::VertexSet& vset) {
  forEachVertex(vset, [](OptimizableGraph::Vertex* v) { v->discardTop(); });
}

void OptimizableGraph::setFixed(HyperGraph::VertexSet& vset, bool fixed) {
  forEachVertex(vset,
                [fixed](OptimizableGraph::Vertex* v) { v->setFixed(fixed); });
}

void OptimizableGraph::forEachVertex(
    std::function<void(OptimizableGraph::Vertex*)> fn) {
  for (auto it = _vertices.begin(); it != _vertices.end(); ++it) {
    OptimizableGraph::Vertex* v =
        static_cast<OptimizableGraph::Vertex*>(it->second);
    fn(v);
  }
}

void OptimizableGraph::forEachVertex(
    HyperGraph::VertexSet& vset,
    std::function<void(OptimizableGraph::Vertex*)> fn) {
  for (auto it = vset.begin(); it != vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    fn(v);
  }
}

bool OptimizableGraph::load(istream& is) {
  set<string> warnedUnknownTypes;
  stringstream currentLine;
  string token;

  Factory* factory = Factory::instance();
  HyperGraph::GraphElemBitset elemBitset;
  elemBitset[HyperGraph::HGET_PARAMETER] = 1;
  elemBitset.flip();

  HyperGraph::GraphElemBitset elemParamBitset;
  elemParamBitset[HyperGraph::HGET_PARAMETER] = 1;

  HyperGraph::DataContainer* previousDataContainer = 0;
  Data* previousData = 0;

  int lineNumber = 0;
  while (1) {
    int bytesRead = readLine(is, currentLine);
    lineNumber++;
    if (bytesRead == -1) break;
    currentLine >> token;
    if (bytesRead == 0 || token.size() == 0 || token[0] == '#') continue;

    // handle commands encoded in the file
    if (token == "FIX") {
      int id;
      while (currentLine >> id) {
        OptimizableGraph::Vertex* v =
            static_cast<OptimizableGraph::Vertex*>(vertex(id));
        if (v) {
#ifndef NDEBUG
          G2O_DEBUG("Fixing vertex {}", v->id());
#endif
          v->setFixed(true);
        } else {
          G2O_WARN("Unable to fix vertex with id {}. Not found in the graph.",
                   id);
        }
      }
      continue;
    }

    // do the mapping to an internal type if it matches
    if (_renamedTypesLookup.size() > 0) {
      map<string, string>::const_iterator foundIt =
          _renamedTypesLookup.find(token);
      if (foundIt != _renamedTypesLookup.end()) {
        token = foundIt->second;
      }
    }

    if (!factory->knowsTag(token)) {
      if (warnedUnknownTypes.count(token) != 1) {
        warnedUnknownTypes.insert(token);
        G2O_ERROR("Unknown type {}", token);
      }
      continue;
    }

    // first handle the parameters
    HyperGraph::HyperGraphElement* pelement =
        factory->construct(token, elemParamBitset);
    if (pelement) {  // not a parameter or otherwise unknown tag
      assert(pelement->elementType() == HyperGraph::HGET_PARAMETER &&
             "Should be a param");
      Parameter* p = static_cast<Parameter*>(pelement);
      int pid;
      currentLine >> pid;
      p->setId(pid);
      bool r = p->read(currentLine);
      if (!r) {
        G2O_ERROR("reading data {} for parameter {} at line ", pid, lineNumber);
        delete p;
      } else {
        if (!_parameters.addParameter(p)) {
          G2O_ERROR("Parameter of type: {} id: {} already defined at line {}",
                    token, pid, lineNumber);
        }
      }
      continue;
    }

    HyperGraph::HyperGraphElement* element =
        factory->construct(token, elemBitset);
    if (dynamic_cast<Vertex*>(element)) {  // it's a vertex type
      previousData = 0;
      Vertex* v = static_cast<Vertex*>(element);
      int id;
      currentLine >> id;
      bool r = v->read(currentLine);
      if (!r)
        G2O_ERROR("Error reading vertex {} {} at line {}", token, id,
                  lineNumber);
      v->setId(id);
      if (!addVertex(v)) {
        G2O_ERROR("Failure adding Vertex {} {} at line {}", token, id,
                  lineNumber);
        delete v;
      } else {
        previousDataContainer = v;
      }
    } else if (dynamic_cast<Edge*>(element)) {
      previousData = 0;
      Edge* e = static_cast<Edge*>(element);
      int numV = e->vertices().size();

      vector<int> ids;
      if (e->vertices().size() != 0) {
        ids.resize(e->vertices().size());
        for (int l = 0; l < numV; ++l) currentLine >> ids[l];
      } else {
        string buff;  // reading the IDs of a dynamically sized edge
        while (currentLine >> buff) {
          if (buff == "||") break;
          ids.push_back(atoi(buff.c_str()));
          currentLine >> buff;
        }
        e->resize(numV);
      }
      bool vertsOkay = true;
      for (size_t l = 0; l < ids.size(); ++l) {
        int vertexId = ids[l];
        if (vertexId != HyperGraph::UnassignedId) {
          HyperGraph::Vertex* v = vertex(vertexId);
          if (!v) {
            vertsOkay = false;
            break;
          }
          e->setVertex(l, v);
        }
      }
      if (!vertsOkay) {
        G2O_ERROR("Unable to find vertices for edge {} at line {} IDs: {}",
                  token, lineNumber, strJoin(ids.begin(), ids.end(), " "));
        delete e;
        e = nullptr;
      } else {
        bool r = e->read(currentLine);
        if (!r || !addEdge(e)) {
          G2O_ERROR("Unable to add edge {} at line {} IDs: {}", token,
                    lineNumber, strJoin(ids.begin(), ids.end(), " "));
          delete e;
          e = nullptr;
        }
      }

      previousDataContainer = e;
    } else if (dynamic_cast<Data*>(
                   element)) {  // reading in the data packet for the vertex
      Data* d = static_cast<Data*>(element);
      bool r = d->read(currentLine);
      if (!r) {
        G2O_ERROR("Error reading data {} at line {}", token, lineNumber);
        delete d;
        previousData = 0;
      } else if (previousData) {
        previousData->setNext(d);
        d->setDataContainer(previousData->dataContainer());
        previousData = d;
      } else if (previousDataContainer) {
        previousDataContainer->setUserData(d);
        d->setDataContainer(previousDataContainer);
        previousData = d;
        previousDataContainer = 0;
      } else {
        G2O_ERROR("got data element, but no data container available");
        delete d;
        previousData = 0;
      }
    }
  }  // while read line

#ifndef NDEBUG
  G2O_DEBUG("Loaded {} parameters", _parameters.size());
#endif

  return true;
}

bool OptimizableGraph::load(const char* filename) {
  ifstream ifs(filename);
  if (!ifs) {
    G2O_ERROR("Unable to open file {}", filename);
    return false;
  }
  return load(ifs);
}

bool OptimizableGraph::save(const char* filename, int level) const {
  ofstream ofs(filename);
  if (!ofs) return false;
  return save(ofs, level);
}

bool OptimizableGraph::save(ostream& os, int level) const {
  // write the parameters to the top of the file
  if (!_parameters.write(os)) return false;
  set<Vertex*, VertexIDCompare> verticesToSave;  // set sorted by ID
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin();
       it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    if (e->level() == level) {
      for (auto it = e->vertices().begin(); it != e->vertices().end(); ++it) {
        if (*it)
          verticesToSave.insert(static_cast<OptimizableGraph::Vertex*>(*it));
      }
    }
  }

  for (auto v : verticesToSave) saveVertex(os, v);

  std::vector<HyperGraph::Edge*> edgesToSave;
  std::copy_if(edges().begin(), edges().end(), std::back_inserter(edgesToSave),
               [level](const HyperGraph::Edge* ee) {
                 const OptimizableGraph::Edge* e =
                     dynamic_cast<const OptimizableGraph::Edge*>(ee);
                 return (e->level() == level);
               });
  sort(edgesToSave.begin(), edgesToSave.end(), EdgeIDCompare());
  for (auto e : edgesToSave) saveEdge(os, static_cast<Edge*>(e));

  return os.good();
}

bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::VertexSet& vset,
                                  int level) {
  if (!_parameters.write(os)) return false;

  for (auto v : vset) saveVertex(os, static_cast<Vertex*>(v));

  for (HyperGraph::EdgeSet::const_iterator it = edges().begin();
       it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(*it);
    if (e->level() != level) continue;

    bool verticesInEdge = true;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin();
         it != e->vertices().end(); ++it) {
      if (vset.find(*it) == vset.end()) {
        verticesInEdge = false;
        break;
      }
    }
    if (!verticesInEdge) continue;

    saveEdge(os, e);
  }

  return os.good();
}

bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::EdgeSet& eset) {
  if (!_parameters.write(os)) return false;
  HyperGraph::VertexSet vset;
  for (auto e : eset)
    for (auto v : e->vertices())
      if (v) vset.insert(v);

  for (auto v : vset) saveVertex(os, static_cast<Vertex*>(v));
  for (auto e : eset) saveEdge(os, static_cast<Edge*>(e));
  return os.good();
}

int OptimizableGraph::maxDimension() const {
  int maxDim = 0;
  for (HyperGraph::VertexIDMap::const_iterator it = vertices().begin();
       it != vertices().end(); ++it) {
    const OptimizableGraph::Vertex* v =
        static_cast<const OptimizableGraph::Vertex*>(it->second);
    maxDim = (std::max)(maxDim, v->dimension());
  }
  return maxDim;
}

void OptimizableGraph::setRenamedTypesFromString(const std::string& types) {
  Factory* factory = Factory::instance();
  vector<string> typesMap = strSplit(types, ",");
  for (size_t i = 0; i < typesMap.size(); ++i) {
    vector<string> m = strSplit(typesMap[i], "=");
    if (m.size() != 2) {
      G2O_ERROR("unable to extract type map from {}", typesMap[i]);
      continue;
    }
    string typeInFile = trim(m[0]);
    string loadedType = trim(m[1]);
    if (!factory->knowsTag(loadedType)) {
      G2O_ERROR("unknown type {}", loadedType);
      continue;
    }

    _renamedTypesLookup[typeInFile] = loadedType;
  }

  G2O_DEBUG("Load look up table:");
  for (std::map<std::string, std::string>::const_iterator it =
           _renamedTypesLookup.begin();
       it != _renamedTypesLookup.end(); ++it) {
    G2O_DEBUG("{} -> {}", it->first, it->second);
  }
}

bool OptimizableGraph::isSolverSuitable(
    const OptimizationAlgorithmProperty& solverProperty,
    const std::set<int>& vertDims_) const {
  std::set<int> auxDims;
  if (vertDims_.size() == 0) {
    auxDims = dimensions();
  }
  const set<int>& vertDims = vertDims_.size() == 0 ? auxDims : vertDims_;
  bool suitableSolver = true;
  if (vertDims.size() == 2) {
    if (solverProperty.requiresMarginalize) {
      suitableSolver = vertDims.count(solverProperty.poseDim) == 1 &&
                       vertDims.count(solverProperty.landmarkDim) == 1;
    } else {
      suitableSolver = solverProperty.poseDim == -1;
    }
  } else if (vertDims.size() == 1) {
    suitableSolver = vertDims.count(solverProperty.poseDim) == 1 ||
                     solverProperty.poseDim == -1;
  } else {
    suitableSolver =
        solverProperty.poseDim == -1 && !solverProperty.requiresMarginalize;
  }
  return suitableSolver;
}

std::set<int> OptimizableGraph::dimensions() const {
  std::set<int> auxDims;
  for (VertexIDMap::const_iterator it = vertices().begin();
       it != vertices().end(); ++it) {
    OptimizableGraph::Vertex* v =
        static_cast<OptimizableGraph::Vertex*>(it->second);
    auxDims.insert(v->dimension());
  }
  return auxDims;
}

void OptimizableGraph::performActions(int iter, HyperGraphActionSet& actions) {
  if (actions.size() > 0) {
    HyperGraphAction::ParametersIteration params(iter);
    for (HyperGraphActionSet::iterator it = actions.begin();
         it != actions.end(); ++it) {
      (*(*it))(this, &params);
    }
  }
}

void OptimizableGraph::preIteration(int iter) {
  performActions(iter, _graphActions[AT_PREITERATION]);
}

void OptimizableGraph::postIteration(int iter) {
  performActions(iter, _graphActions[AT_POSTITERATION]);
}

bool OptimizableGraph::addPostIterationAction(HyperGraphAction* action) {
  std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      _graphActions[AT_POSTITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::addPreIterationAction(HyperGraphAction* action) {
  std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      _graphActions[AT_PREITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::removePreIterationAction(HyperGraphAction* action) {
  return _graphActions[AT_PREITERATION].erase(action) > 0;
}

bool OptimizableGraph::removePostIterationAction(HyperGraphAction* action) {
  return _graphActions[AT_POSTITERATION].erase(action) > 0;
}

bool OptimizableGraph::saveUserData(std::ostream& os,
                                    HyperGraph::Data* d) const {
  Factory* factory = Factory::instance();
  while (d) {  // write the data packet for the vertex
    string tag = factory->tag(d);
    if (tag.size() > 0) {
      os << tag << " ";
      d->write(os);
      os << endl;
    }
    d = d->next();
  }
  return os.good();
}

bool OptimizableGraph::saveVertex(std::ostream& os,
                                  OptimizableGraph::Vertex* v) const {
  Factory* factory = Factory::instance();
  string tag = factory->tag(v);
  if (tag.size() > 0) {
    os << tag << " " << v->id() << " ";
    v->write(os);
    os << endl;
    saveUserData(os, v->userData());
    if (v->fixed()) {
      os << "FIX " << v->id() << endl;
    }
    return os.good();
  }
  return false;
}

bool OptimizableGraph::saveParameter(std::ostream& os, Parameter* p) const {
  Factory* factory = Factory::instance();
  string tag = factory->tag(p);
  if (tag.size() > 0) {
    os << tag << " " << p->id() << " ";
    p->write(os);
    os << endl;
  }
  return os.good();
}

bool OptimizableGraph::saveEdge(std::ostream& os,
                                OptimizableGraph::Edge* e) const {
  Factory* factory = Factory::instance();
  string tag = factory->tag(e);
  if (tag.size() > 0) {
    os << tag << " ";
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin();
         it != e->vertices().end(); ++it) {
      int vertexId = (*it) ? (*it)->id() : HyperGraph::UnassignedId;
      os << vertexId << " ";
    }
    e->write(os);
    os << endl;
    saveUserData(os, e->userData());
    return os.good();
  }
  return false;
}

void OptimizableGraph::clearParameters() {
  HyperGraph::clear();
  _parameters.clear();
}

bool OptimizableGraph::verifyInformationMatrices(bool verbose) const {
  bool allEdgeOk = true;
  Eigen::SelfAdjointEigenSolver<MatrixX> eigenSolver;
  for (OptimizableGraph::EdgeSet::const_iterator it = edges().begin();
       it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    MatrixX::MapType information(e->informationData(), e->dimension(),
                                 e->dimension());
    // test on symmetry
    bool isSymmetric = information.transpose() == information;
    bool okay = isSymmetric;
    if (isSymmetric) {
      // compute the eigenvalues
      eigenSolver.compute(information, Eigen::EigenvaluesOnly);
      bool isSPD = eigenSolver.eigenvalues()(0) >= 0.;
      okay = okay && isSPD;
    }
    allEdgeOk = allEdgeOk && okay;
    if (!okay) {
      if (verbose) {
        vector<int> ids(e->vertices().size());
        for (size_t i = 0; i < e->vertices().size(); ++i)
          ids[i] = e->vertex(i)->id();
        if (!isSymmetric)
          G2O_WARN("Information Matrix for an edge is not symmetric: {}",
                   strJoin(ids.begin(), ids.end(), " "));
        else
          G2O_WARN("Information Matrix for an edge is not SPD: {}",
                   strJoin(ids.begin(), ids.end(), " "));
        if (isSymmetric)
          G2O_WARN("eigenvalues: {}", eigenSolver.eigenvalues().transpose());
      }
    }
  }
  return allEdgeOk;
}

bool OptimizableGraph::initMultiThreading() {
#if (defined G2O_OPENMP) && EIGEN_VERSION_AT_LEAST(3, 1, 0)
  Eigen::initParallel();
#endif
  return true;
}

}  // namespace g2o
