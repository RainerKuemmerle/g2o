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
#include <iomanip>
#include <iostream>
#include <iterator>
#include <utility>

#include "cache.h"
#include "estimate_propagator.h"
#include "factory.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/string_tools.h"
#include "hyper_graph_action.h"
#include "optimization_algorithm_property.h"
#include "robust_kernel.h"

namespace g2o {

static std::shared_ptr<OptimizableGraph::Vertex> kNonExistantVertex(nullptr);

using std::cerr;
using std::endl;
using std::string;
using std::vector;

namespace {
std::ostream& printIdChain(std::ostream& os, const std::vector<int>& ids) {
  for (size_t l = 0; l < ids.size(); ++l) {
    if (l > 0) cerr << " <->";
    cerr << " " << ids[l];
  }
  return os;
}
}  // namespace

std::shared_ptr<CacheContainer> OptimizableGraph::Vertex::cacheContainer() {
  if (!cacheContainer_) {
    cacheContainer_ = std::make_shared<CacheContainer>(*this);
  }
  return cacheContainer_;
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

bool OptimizableGraph::Vertex::setEstimateData(const number_t* estimate) {
  const bool ret = setEstimateDataImpl(estimate);
  updateCache();
  return ret;
}

bool OptimizableGraph::Vertex::getEstimateData(number_t*) const {
  return false;
}

int OptimizableGraph::Vertex::estimateDimension() const { return -1; }

bool OptimizableGraph::Vertex::setMinimalEstimateData(
    const number_t* estimate) {
  const bool ret = setMinimalEstimateDataImpl(estimate);
  updateCache();
  return ret;
}

bool OptimizableGraph::Vertex::getMinimalEstimateData(number_t*) const {
  return false;
}

int OptimizableGraph::Vertex::minimalEstimateDimension() const { return -1; }

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
    cerr << __PRETTY_FUNCTION__ << ": edge not registered with a graph" << endl;
    return false;
  }

  assert(parameters_.size() == parameterIds_.size());
  // cerr << __PRETTY_FUNCTION__ << ": encountered " << _parameters.size() << "
  // parameters" << endl;
  for (size_t i = 0; i < parameters_.size(); i++) {
    const int index = parameterIds_[i];
    parameters_[i] = graph()->parameter(index);
#ifndef NDEBUG
    auto& aux = *parameters_[i];
    if (typeid(aux).name() != parameterTypes_[i]) {
      cerr << __PRETTY_FUNCTION__
           << ": FATAL, parameter type mismatch - encountered "
           << typeid(aux).name() << "; should be " << parameterTypes_[i]
           << endl;
    }
#endif
    if (!parameters_[i]) {
      cerr << __PRETTY_FUNCTION__ << ": FATAL, _parameters[i] == nullptr"
           << endl;
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

bool OptimizableGraph::Edge::setMeasurementData(const number_t*) {
  return false;
}

bool OptimizableGraph::Edge::getMeasurementData(number_t*) const {
  return false;
}

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
    cerr << __FUNCTION__ << ": FATAL, a vertex with (negative) ID " << ov->id()
         << " cannot be inserted in the graph" << endl;
    assert(0 && "Invalid vertex id");
    return false;
  }
  if (vertex(ov->id())) {
    cerr << __FUNCTION__ << ": FATAL, a vertex with ID " << ov->id()
         << " has already been registered with this graph" << endl;
    return false;
  }
  if (ov->graph_ != nullptr && ov->graph_ != this) {
    cerr << __FUNCTION__ << ": FATAL, vertex with ID " << ov->id()
         << " has already registered with another graph " << ov->graph_ << endl;
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
    cerr << __FUNCTION__ << ": FATAL, edge with ID " << e->id()
         << " has already registered with another graph " << g << endl;
    return false;
  }

  const bool eresult = HyperGraph::addEdge(he);
  if (!eresult) return false;

  e->internalId_ = nextEdgeId_++;
  if (e->numUndefinedVertices()) return true;
  if (!e->resolveParameters()) {
    cerr << __FUNCTION__ << ": FATAL, cannot resolve parameters for edge " << e
         << endl;
    return false;
  }
  if (!e->resolveCaches()) {
    cerr << __FUNCTION__ << ": FATAL, cannot resolve caches for edge " << e
         << endl;
    return false;
  }

  jacobianWorkspace_.updateSize(e);

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
      cerr << __FUNCTION__ << ": FATAL, cannot resolve parameters for edge "
           << e << endl;
      return false;
    }
    if (!ee->resolveCaches()) {
      cerr << __FUNCTION__ << ": FATAL, cannot resolve caches for edge " << e
           << endl;
      return false;
    }
    jacobianWorkspace_.updateSize(e.get());
  }
  return true;
}

int OptimizableGraph::optimize(int /*iterations*/, bool /*online*/) {
  return -1;
}

number_t OptimizableGraph::chi2() const {
  number_t chi = 0.0;
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
    // cerr << "Token=" << token << endl;
    if (bytesRead == 0 || token.empty() || token[0] == '#') continue;

    // handle commands encoded in the file
    if (token == "FIX") {
      int id;
      while (currentLine >> id) {
        auto v = vertex(id);
        if (v) {
#ifndef NDEBUG
          cerr << "Fixing vertex " << v->id() << endl;
#endif
          v->setFixed(true);
        } else {
          cerr << "Warning: Unable to fix vertex with id " << id
               << ". Not found in the graph." << endl;
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
        cerr << CL_RED(__PRETTY_FUNCTION__ << " unknown type: " << token)
             << endl;
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
        cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token
             << " for parameter " << pid << " at line " << lineNumber << endl;
      } else {
        if (!parameters_.addParameter(p)) {
          cerr << __PRETTY_FUNCTION__ << ": Parameter of type:" << token
               << " id:" << pid << " already defined"
               << " at line " << lineNumber << endl;
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
        cerr << __PRETTY_FUNCTION__ << ": Error reading vertex " << token << " "
             << id << " at line " << lineNumber << endl;
      v->setId(id);
      if (!addVertex(v)) {
        cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex, " << token
             << " " << id << " at line " << lineNumber << endl;
      } else {
        previousDataContainer = v;
      }
    } else if (dynamic_cast<Edge*>(element.get())) {
      // cerr << "it is an edge" << endl;
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
          if (buff == "||") break;
          ids.push_back(atoi(buff.c_str()));
          currentLine >> buff;
        }
        e->resize(numV);
      }
      bool vertsOkay = true;
      for (size_t l = 0; l < ids.size(); ++l) {
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
        cerr << __PRETTY_FUNCTION__ << ": Unable to find vertices for edge "
             << token << " at line " << lineNumber << " IDs: ";
        printIdChain(cerr, ids) << std::endl;
        e = nullptr;
      } else {
        const bool r = e->read(currentLine);
        if (!r || !addEdge(e)) {
          cerr << __PRETTY_FUNCTION__ << ": Unable to add edge " << token
               << " at line " << lineNumber << " IDs: ";
          printIdChain(cerr, ids) << std::endl;
          e = nullptr;
        }
      }

      previousDataContainer = e;
    } else if (dynamic_cast<Data*>(element.get())) {  // reading in the data
                                                      // packet for the vertex
      // cerr << "read data packet " << token << " vertex " <<
      // previousVertex->id() << endl;
      auto d = std::static_pointer_cast<Data>(element);
      const bool r = d->read(currentLine);
      if (!r) {
        cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token
             << " at line " << lineNumber << " IDs: " << endl;
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
        cerr << __PRETTY_FUNCTION__
             << ": got data element, but no data container available" << endl;
        previousData = nullptr;
      }
    }
  }  // while read line

#ifndef NDEBUG
  cerr << "Loaded " << parameters_.size() << " parameters" << endl;
#endif

  return true;
}

bool OptimizableGraph::load(const char* filename) {
  std::ifstream ifs(filename);
  if (!ifs) {
    cerr << __PRETTY_FUNCTION__ << " unable to open file " << filename << endl;
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
      cerr << __PRETTY_FUNCTION__ << ": unable to extract type map from " << i
           << endl;
      continue;
    }
    const string typeInFile = trim(m[0]);
    const string loadedType = trim(m[1]);
    if (!factory->knowsTag(loadedType)) {
      cerr << __PRETTY_FUNCTION__ << ": unknown type " << loadedType << endl;
      continue;
    }

    renamedTypesLookup_[typeInFile] = loadedType;
  }

  cerr << "# load look up table" << endl;
  for (std::map<std::string, std::string>::const_iterator it =
           renamedTypesLookup_.begin();
       it != renamedTypesLookup_.end(); ++it) {
    cerr << "#\t" << it->first << " -> " << it->second << endl;
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
      os << endl;
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
    os << endl;
    saveUserData(os, v->userData().get());
    if (v->fixed()) {
      os << "FIX " << v->id() << endl;
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
    os << endl;
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
    os << endl;
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
        if (!isSymmetric)
          cerr << "Information Matrix for an edge is not symmetric:";
        else
          cerr << "Information Matrix for an edge is not SPD:";
        for (size_t i = 0; i < e->vertices().size(); ++i)
          cerr << " " << e->vertex(i)->id();
        if (isSymmetric)
          cerr << "\teigenvalues: " << eigenSolver.eigenvalues().transpose();
        cerr << endl;
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
