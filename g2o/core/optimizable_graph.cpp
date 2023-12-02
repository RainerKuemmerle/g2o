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

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <utility>

#include "cache.h"
#include "factory.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/parameter.h"
#include "g2o/core/parameter_container.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/logger_format.h"  // IWYU pragma: keep
#include "g2o/stuff/string_tools.h"
#include "hyper_graph_action.h"
#include "optimization_algorithm_property.h"

namespace g2o {

namespace {
std::shared_ptr<OptimizableGraph::Vertex> kNonExistantVertex(nullptr);
}

using std::string;
using std::vector;

// Here to destruct forward declared types
OptimizableGraph::Vertex::Vertex() = default;
OptimizableGraph::Vertex::~Vertex() = default;

CacheContainer& OptimizableGraph::Vertex::cacheContainer() {
  if (!cacheContainer_) {
    cacheContainer_ = std::make_unique<CacheContainer>(*this);
  }
  return *cacheContainer_;
}

MatrixX::MapType OptimizableGraph::Vertex::hessianMap() const {
  const int dim = dimension();
  return MatrixN<Eigen::Dynamic>::MapType(hessianData(), dim, dim);
}

VectorX::MapType OptimizableGraph::Vertex::bMap() const {
  const int dim = dimension();
  return VectorX::MapType(bData(), dim);
}

void OptimizableGraph::Vertex::updateCache() {
  if (cacheContainer_) {
    cacheContainer_->setUpdateNeeded();
    cacheContainer_->update();
  }
}

OptimizableGraph::Edge::Edge() : robustKernel_(nullptr) {}

OptimizableGraph::Edge::~Edge() = default;

OptimizableGraph* OptimizableGraph::Edge::graph() {
  if (vertices_.empty()) return nullptr;
  auto& v = vertices_[0];
  if (!v) return nullptr;
  return static_cast<OptimizableGraph::Vertex*>(v.get())->graph();
}

const OptimizableGraph* OptimizableGraph::Edge::graph() const {
  if (vertices_.empty()) return nullptr;
  const auto& v = vertices_[0];
  if (!v) return nullptr;
  return static_cast<OptimizableGraph::Vertex*>(v.get())->graph();
}

bool OptimizableGraph::Edge::setParameterId(int argNum, int paramId) {
  if (static_cast<int>(parameters_.size()) <= argNum) return false;
  if (argNum < 0) return false;
  parameters_[argNum] = nullptr;
  parameterIds_[argNum] = paramId;
  return true;
}

bool OptimizableGraph::Edge::resolveParameters() {
  if (!graph()) {
    G2O_ERROR("{}: edge not registered with a graph", __PRETTY_FUNCTION__);
    return false;
  }

  assert(parameters_.size() == parameterIds_.size());
  for (decltype(parameters_)::size_type i = 0; i < parameters_.size(); i++) {
    const int index = parameterIds_[i];
    parameters_[i] = graph()->parameter(index);
#ifndef NDEBUG
    auto& aux = *parameters_[i];
    if (typeid(aux).name() != parameterTypes_[i]) {
      G2O_ERROR(
          "{}: FATAL, parameter type mismatch - encountered {}; should be {}",
          __PRETTY_FUNCTION__, typeid(aux).name(), parameterTypes_[i]);
    }
#endif
    if (!parameters_[i]) {
      G2O_ERROR("{}: FATAL, _parameters[i] == nullptr", __PRETTY_FUNCTION__);
      return false;
    }
  }
  return true;
}

void OptimizableGraph::Edge::setRobustKernel(
    std::shared_ptr<RobustKernel> ptr) {
  robustKernel_ = std::move(ptr);
}

bool OptimizableGraph::Edge::resolveCaches() { return true; }

bool OptimizableGraph::Edge::setMeasurementData(const double*) { return false; }

bool OptimizableGraph::Edge::getMeasurementData(double*) const { return false; }

int OptimizableGraph::Edge::measurementDimension() const { return -1; }

bool OptimizableGraph::Edge::setMeasurementFromState() { return false; }

OptimizableGraph::OptimizableGraph() {
  nextEdgeId_ = 0;
  graphActions_.resize(kAtNumElements);
}

OptimizableGraph::~OptimizableGraph() {
  clear();
  clearParameters();
}

bool OptimizableGraph::addVertex(
    const std::shared_ptr<HyperGraph::Vertex>& v,
    const std::shared_ptr<HyperGraph::Data>& userData) {
  auto* ov = dynamic_cast<OptimizableGraph::Vertex*>(v.get());
  if (!ov) return false;
  if (ov->id() < 0) {
    G2O_ERROR(
        "{}: FATAL, a vertex with (negative) ID {} cannot be inserted into the "
        "graph",
        __PRETTY_FUNCTION__, ov->id());
    assert(0 && "Invalid vertex id");
    return false;
  }
  if (vertex(ov->id())) {
    G2O_WARN(
        "{}: a vertex with ID {} has already been registered with this "
        "graph",
        __PRETTY_FUNCTION__, ov->id());
    return false;
  }
  if (ov->graph_ != nullptr && ov->graph_ != this) {
    G2O_ERROR(
        "{}: FATAL, vertex with ID {} has already been registered with another "
        "graph {}",
        __PRETTY_FUNCTION__, ov->id(), static_cast<void*>(ov->graph_));
    return false;
  }
  if (userData) ov->setUserData(userData);
  ov->graph_ = this;
  return HyperGraph::addVertex(v);
}

bool OptimizableGraph::removeVertex(
    const std::shared_ptr<HyperGraph::Vertex>& v, bool detach) {
  auto* ov = dynamic_cast<OptimizableGraph::Vertex*>(v.get());
  if (ov && ov->graph_ == this) ov->graph_ = nullptr;
  return HyperGraph::removeVertex(v, detach);
}

bool OptimizableGraph::addEdge(const std::shared_ptr<HyperGraph::Edge>& he) {
  auto* e = dynamic_cast<OptimizableGraph::Edge*>(he.get());
  if (!e) return false;
  OptimizableGraph* g = e->graph();

  if (g != nullptr && g != this) {
    G2O_ERROR(
        "{}: FATAL, edge with ID {} has already registered with another graph "
        "{}",
        __PRETTY_FUNCTION__, e->id(), static_cast<void*>(g));
    return false;
  }

  const bool eresult = HyperGraph::addEdge(he);
  if (!eresult) return false;

  e->internalId_ = nextEdgeId_++;
  if (e->numUndefinedVertices()) return true;
  if (!e->resolveParameters()) {
    G2O_ERROR("{}: FATAL, cannot resolve parameters for edge {}",
              static_cast<void*>(e));
    return false;
  }
  if (!e->resolveCaches()) {
    G2O_ERROR("{}: FATAL, cannot resolve caches for edge {}",
              static_cast<void*>(e));
    return false;
  }

  jacobianWorkspace_.updateSize(*e);

  return true;
}

std::shared_ptr<OptimizableGraph::Vertex> OptimizableGraph::vertex(int id) {
  return std::static_pointer_cast<Vertex>(HyperGraph::vertex(id));
}

std::shared_ptr<const OptimizableGraph::Vertex> OptimizableGraph::vertex(
    int id) const {
  return std::static_pointer_cast<const OptimizableGraph::Vertex>(
      HyperGraph::vertex(id));
}

// bool OptimizableGraph::addEdge(const std::shared_ptr<HyperGraph::Edge>& e_) {
//   std::shared_ptr<OptimizableGraph::Edge> e =
//   dynamic_pointer_cast<OptimizableGraph::Edge>(e_); assert(e && "Edge does
//   not inherit from OptimizableGraph::Edge"); if (!e) return false; return
//   addEdge(e);
// }

bool OptimizableGraph::setEdgeVertex(
    const std::shared_ptr<HyperGraph::Edge>& e, int pos,
    const std::shared_ptr<HyperGraph::Vertex>& v) {
  if (!HyperGraph::setEdgeVertex(e, pos, v)) {
    return false;
  }
  if (!e->numUndefinedVertices()) {
#ifndef NDEBUG
    auto ee = std::dynamic_pointer_cast<OptimizableGraph::Edge>(e);
    assert(ee && "Edge is not a OptimizableGraph::Edge");
#else
    auto ee = std::static_pointer_cast<OptimizableGraph::Edge>(e);
#endif
    if (!ee->resolveParameters()) {
      G2O_ERROR("{}: FATAL, cannot resolve parameters for edge {}",
                static_cast<void*>(e.get()));
      return false;
    }
    if (!ee->resolveCaches()) {
      G2O_ERROR("{}: FATAL, cannot resolve caches for edge {}",
                static_cast<void*>(e.get()));
      return false;
    }
    jacobianWorkspace_.updateSize(*e);
  }
  return true;
}

int OptimizableGraph::optimize(int /*iterations*/, bool /*online*/) {
  return -1;
}

double OptimizableGraph::chi2() const {
  double chi = 0.0;
  for (const auto& it : this->edges()) {
    const auto* e = static_cast<const OptimizableGraph::Edge*>(it.get());
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
    const std::function<void(OptimizableGraph::Vertex*)>& fn) {
  for (auto& vertex : vertices_) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(vertex.second.get());
    fn(v);
  }
}

void OptimizableGraph::forEachVertex(
    HyperGraph::VertexSet& vset,
    const std::function<void(OptimizableGraph::Vertex*)>& fn) {
  for (const auto& it : vset) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.get());
    fn(v);
  }
}

bool OptimizableGraph::load(std::istream& is) {
  std::set<string> warnedUnknownTypes;
  std::stringstream currentLine;
  string token;

  Factory* factory = Factory::instance();
  HyperGraph::GraphElemBitset elemBitset;
  elemBitset[HyperGraph::kHgetParameter] = true;
  elemBitset.flip();

  HyperGraph::GraphElemBitset elemParamBitset;
  elemParamBitset[HyperGraph::kHgetParameter] = true;

  std::shared_ptr<HyperGraph::DataContainer> previousDataContainer;
  Data* previousData = nullptr;

  int lineNumber = 0;
  while (true) {
    const int bytesRead = readLine(is, currentLine);
    lineNumber++;
    if (bytesRead == -1) break;
    currentLine >> token;
    if (bytesRead == 0 || token.empty() || token[0] == '#') continue;

    // handle commands encoded in the file
    if (token == "FIX") {
      int id;
      while (currentLine >> id) {
        auto v = vertex(id);
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
    if (!renamedTypesLookup_.empty()) {
      auto foundIt = renamedTypesLookup_.find(token);
      if (foundIt != renamedTypesLookup_.end()) {
        token = foundIt->second;
      }
    }

    if (!factory->knowsTag(token)) {
      if (warnedUnknownTypes.count(token) != 1) {
        warnedUnknownTypes.insert(token);
        G2O_ERROR("{}: Unknown type {}", __PRETTY_FUNCTION__, token);
      }
      continue;
    }

    // first handle the parameters
    const std::shared_ptr<HyperGraph::HyperGraphElement> pelement =
        factory->construct(token, elemParamBitset);
    if (pelement) {  // not a parameter or otherwise unknown tag
      assert(pelement->elementType() == HyperGraph::kHgetParameter &&
             "Should be a param");
      auto p = std::static_pointer_cast<Parameter>(pelement);
      int pid;
      currentLine >> pid;
      p->setId(pid);
      const bool r = p->read(currentLine);
      if (!r) {
        G2O_ERROR("{}: reading data {} for parameter {} at line ",
                  __PRETTY_FUNCTION__, pid, lineNumber);
      } else {
        if (!parameters_.addParameter(p)) {
          G2O_ERROR(
              "{}: Parameter of type: {} id: {} already defined at line {}",
              __PRETTY_FUNCTION__, token, pid, lineNumber);
        }
      }
      continue;
    }

    const std::shared_ptr<HyperGraph::HyperGraphElement> element =
        factory->construct(token, elemBitset);
    if (dynamic_cast<Vertex*>(element.get())) {  // it's a vertex type
      previousData = nullptr;
      auto v = std::static_pointer_cast<Vertex>(element);
      int id;
      currentLine >> id;
      const bool r = v->read(currentLine);
      if (!r)
        G2O_ERROR("{}: Error reading vertex {} {} at line {}",
                  __PRETTY_FUNCTION__, token, id, lineNumber);
      v->setId(id);
      if (!addVertex(v)) {
        G2O_ERROR("{}: Failure adding Vertex {} {} at line {}",
                  __PRETTY_FUNCTION__, token, id, lineNumber);
      } else {
        previousDataContainer = v;
      }
    } else if (dynamic_cast<Edge*>(element.get())) {
      previousData = nullptr;
      auto e = std::static_pointer_cast<Edge>(element);
      const int numV = e->vertices().size();

      vector<int> ids;
      if (!e->vertices().empty()) {
        ids.resize(e->vertices().size());
        for (int l = 0; l < numV; ++l) currentLine >> ids[l];
      } else {
        string buff;  // reading the IDs of a dynamically sized edge
        while (currentLine >> buff) {
          // TODO(rainer): reading/writing multi dynamically sized edges is a
          // bad design. Get rid of writing || in the edges
          if (buff == "||") break;
          ids.push_back(stoi(buff));
          currentLine >> buff;
        }
        e->resize(numV);
      }
      bool vertsOkay = true;
      for (vector<int>::size_type l = 0; l < ids.size(); ++l) {
        const int vertexId = ids[l];
        if (vertexId != HyperGraph::kUnassignedId) {
          auto v = vertex(vertexId);
          if (!v) {
            vertsOkay = false;
            break;
          }
          e->setVertex(l, v);
        }
      }
      if (!vertsOkay) {
        G2O_ERROR("{}: Unable to find vertices for edge {} at line {} IDs: {}",
                  __PRETTY_FUNCTION__, token, lineNumber, fmt::join(ids, " "));
        e = nullptr;
      } else {
        const bool r = e->read(currentLine);
        if (!r || !addEdge(e)) {
          G2O_ERROR("{}: Unable to add edge {} at line {} IDs: {}",
                    __PRETTY_FUNCTION__, token, lineNumber,
                    fmt::join(ids, " "));
          e = nullptr;
        }
      }

      previousDataContainer = e;
    } else if (dynamic_cast<Data*>(element.get())) {  // reading in the data
                                                      // packet for the vertex
      auto d = std::static_pointer_cast<Data>(element);
      const bool r = d->read(currentLine);
      if (!r) {
        G2O_ERROR("{}: Error reading data {} at line {}", __PRETTY_FUNCTION__,
                  token, lineNumber);
        previousData = nullptr;
      } else if (previousData) {
        previousData->setNext(d);
        d->setDataContainer(previousData->dataContainer());
        previousData = d.get();
      } else if (previousDataContainer) {
        previousDataContainer->setUserData(d);
        d->setDataContainer(previousDataContainer);
        previousData = d.get();
        previousDataContainer = nullptr;
      } else {
        G2O_ERROR("{}: got data element, but no data container available",
                  __PRETTY_FUNCTION__);
        previousData = nullptr;
      }
    }
  }  // while read line

#ifndef NDEBUG
  G2O_DEBUG("Loaded {} parameters", parameters_.size());
#endif

  return true;
}

bool OptimizableGraph::load(const char* filename) {
  std::ifstream ifs(filename);
  if (!ifs) {
    G2O_ERROR("Unable to open file {}", filename);
    return false;
  }
  return load(ifs);
}

bool OptimizableGraph::save(const char* filename, int level) const {
  std::ofstream ofs(filename);
  if (!ofs) return false;
  return save(ofs, level);
}

bool OptimizableGraph::save(std::ostream& os, int level) const {
  // write the parameters to the top of the file
  if (!parameters_.write(os)) return false;
  std::set<Vertex*, VertexIDCompare> verticesToSave;  // set sorted by ID
  for (const auto& it : edges()) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    if (e->level() == level) {
      for (auto& it : e->vertices()) {
        if (it)
          verticesToSave.insert(
              static_cast<OptimizableGraph::Vertex*>(it.get()));
      }
    }
  }

  for (auto* v : verticesToSave) saveVertex(os, v);

  std::vector<std::shared_ptr<HyperGraph::Edge>> edgesToSave;
  std::copy_if(edges().begin(), edges().end(), std::back_inserter(edgesToSave),
               [level](const std::shared_ptr<HyperGraph::Edge>& ee) {
                 auto* e = dynamic_cast<OptimizableGraph::Edge*>(ee.get());
                 return (e->level() == level);
               });
  sort(edgesToSave.begin(), edgesToSave.end(), EdgeIDCompare());
  for (const auto& e : edgesToSave) saveEdge(os, static_cast<Edge*>(e.get()));

  return os.good();
}

bool OptimizableGraph::saveSubset(std::ostream& os, HyperGraph::VertexSet& vset,
                                  int level) {
  if (!parameters_.write(os)) return false;

  for (const auto& v : vset) saveVertex(os, static_cast<Vertex*>(v.get()));

  for (const auto& it : edges()) {
    auto* e = dynamic_cast<OptimizableGraph::Edge*>(it.get());
    if (e->level() != level) continue;
    bool verticesInEdge = true;
    for (auto& it : e->vertices()) {
      if (vset.find(it) == vset.end()) {
        verticesInEdge = false;
        break;
      }
    }
    if (!verticesInEdge) continue;
    saveEdge(os, e);
  }

  return os.good();
}

bool OptimizableGraph::saveSubset(std::ostream& os, HyperGraph::EdgeSet& eset) {
  if (!parameters_.write(os)) return false;
  HyperGraph::VertexSet vset;
  for (const auto& e : eset)
    for (const auto& v : e->vertices())
      if (v) vset.insert(v);

  for (const auto& v : vset) saveVertex(os, static_cast<Vertex*>(v.get()));
  for (const auto& e : eset) saveEdge(os, static_cast<Edge*>(e.get()));
  return os.good();
}

int OptimizableGraph::maxDimension() const {
  int maxDim = 0;
  for (const auto& it : vertices()) {
    const auto* v =
        static_cast<const OptimizableGraph::Vertex*>(it.second.get());
    maxDim = (std::max)(maxDim, v->dimension());
  }
  return maxDim;
}

void OptimizableGraph::setRenamedTypesFromString(const std::string& types) {
  Factory* factory = Factory::instance();
  const vector<string> typesMap = strSplit(types, ",");
  for (const auto& i : typesMap) {
    vector<string> m = strSplit(i, "=");
    if (m.size() != 2) {
      G2O_ERROR("{}: unable to extract type map from {}", __PRETTY_FUNCTION__,
                i);
      continue;
    }
    const string typeInFile = trim(m[0]);
    const string loadedType = trim(m[1]);
    if (!factory->knowsTag(loadedType)) {
      G2O_ERROR("{}: unknown type {}", __PRETTY_FUNCTION__, loadedType);
      continue;
    }

    renamedTypesLookup_[typeInFile] = loadedType;
  }

  G2O_DEBUG("Load look up table:");
  for (auto it = renamedTypesLookup_.begin(); it != renamedTypesLookup_.end();
       ++it) {
    G2O_DEBUG("{} -> {}", it->first, it->second);
  }
}

bool OptimizableGraph::isSolverSuitable(
    const OptimizationAlgorithmProperty& solverProperty,
    const std::set<int>& vertDims_) const {
  std::set<int> auxDims;
  if (vertDims_.empty()) {
    auxDims = dimensions();
  }
  const std::set<int>& vertDims = vertDims_.empty() ? auxDims : vertDims_;
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
  for (const auto& it : vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    auxDims.insert(v->dimension());
  }
  return auxDims;
}

void OptimizableGraph::performActions(int iter, HyperGraphActionSet& actions) {
  if (!actions.empty()) {
    auto params = std::make_shared<HyperGraphAction::ParametersIteration>(iter);
    for (const auto& action : actions) {
      (*action)(*this, params);
    }
  }
}

void OptimizableGraph::preIteration(int iter) {
  performActions(iter, graphActions_[kAtPreiteration]);
}

void OptimizableGraph::postIteration(int iter) {
  performActions(iter, graphActions_[kAtPostiteration]);
}

bool OptimizableGraph::addPostIterationAction(
    std::shared_ptr<HyperGraphAction> action) {
  const std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      graphActions_[kAtPostiteration].emplace(action);
  return insertResult.second;
}

bool OptimizableGraph::addPreIterationAction(
    std::shared_ptr<HyperGraphAction> action) {
  const std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      graphActions_[kAtPreiteration].emplace(action);
  return insertResult.second;
}

bool OptimizableGraph::removePreIterationAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  return graphActions_[kAtPreiteration].erase(action) > 0;
}

bool OptimizableGraph::removePostIterationAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  return graphActions_[kAtPostiteration].erase(action) > 0;
}

bool OptimizableGraph::saveUserData(std::ostream& os, HyperGraph::Data* d) {
  Factory* factory = Factory::instance();
  while (d) {  // write the data packet for the vertex
    const string tag = factory->tag(d);
    if (!tag.empty()) {
      os << tag << " ";
      d->write(os);
      os << '\n';
    }
    d = d->next().get();
  }
  return os.good();
}

bool OptimizableGraph::saveVertex(std::ostream& os,
                                  OptimizableGraph::Vertex* v) {
  Factory* factory = Factory::instance();
  const string tag = factory->tag(v);
  if (!tag.empty()) {
    os << tag << " " << v->id() << " ";
    v->write(os);
    os << '\n';
    saveUserData(os, v->userData().get());
    if (v->fixed()) {
      os << "FIX " << v->id() << '\n';
    }
    return os.good();
  }
  return false;
}

bool OptimizableGraph::saveParameter(std::ostream& os, Parameter* p) {
  Factory* factory = Factory::instance();
  const string tag = factory->tag(p);
  if (!tag.empty()) {
    os << tag << " " << p->id() << " ";
    p->write(os);
    os << '\n';
  }
  return os.good();
}

bool OptimizableGraph::saveEdge(std::ostream& os, OptimizableGraph::Edge* e) {
  Factory* factory = Factory::instance();
  const string tag = factory->tag(e);
  if (!tag.empty()) {
    os << tag << " ";
    for (auto& it : e->vertices()) {
      const int vertexId = it ? it->id() : HyperGraph::kUnassignedId;
      os << vertexId << " ";
    }
    e->write(os);
    os << '\n';
    saveUserData(os, e->userData().get());
    return os.good();
  }
  return false;
}

void OptimizableGraph::clear() {
  for (auto& vptr : vertices_) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(vptr.second.get());
    v->graph_ = nullptr;
  }
  HyperGraph::clear();
}

void OptimizableGraph::clearParameters() {
  clear();
  parameters_.clear();
}

bool OptimizableGraph::verifyInformationMatrices(bool verbose) const {
  bool allEdgeOk = true;
  Eigen::SelfAdjointEigenSolver<MatrixX> eigenSolver;
  for (const auto& it : edges()) {
    auto* e = static_cast<OptimizableGraph::Edge*>(it.get());
    MatrixX::MapType information(e->informationData(), e->dimension(),
                                 e->dimension());
    // test on symmetry
    const bool isSymmetric = information.transpose() == information;
    bool okay = isSymmetric;
    if (isSymmetric) {
      // compute the eigenvalues
      eigenSolver.compute(information, Eigen::EigenvaluesOnly);
      const bool isSPD = eigenSolver.eigenvalues()(0) >= 0.;
      okay = okay && isSPD;
    }
    allEdgeOk = allEdgeOk && okay;
    if (!okay) {
      if (verbose) {
        vector<int> ids;
        ids.reserve(e->vertices().size());
        for (const auto& v : e->vertices()) ids.push_back(v->id());
        if (!isSymmetric)
          G2O_WARN("Information Matrix for an edge is not symmetric: {}",
                   fmt::join(ids, " "));
        else
          G2O_WARN("Information Matrix for an edge is not SPD: {}",
                   fmt::join(ids, " "));
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
