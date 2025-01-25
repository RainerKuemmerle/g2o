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
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include "cache.h"
#include "factory.h"
#include "g2o/config.h"  // IWYU pragma: keep
#include "g2o/core/abstract_graph.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/io/io_format.h"
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

void saveUserData(AbstractGraph::AbstractGraphElement& graph_element,
                  const HyperGraph::DataContainer::DataVector& data) {
  if (data.empty()) return;
  Factory* factory = Factory::instance();

  // write the data packet for the vertex
  for (const auto& d : data) {
    const std::string tag = factory->tag(d.get());
    if (tag.empty()) continue;
    std::stringstream buffer;
    d->write(buffer);
    graph_element.data.emplace_back(tag, buffer.str());
  }
}

bool saveParameter(AbstractGraph& abstract_graph, Parameter* p) {
  Factory* factory = Factory::instance();
  const std::string tag = factory->tag(p);
  if (tag.empty()) return false;
  std::vector<double> data;
  p->getParameterData(data);
  abstract_graph.parameters().emplace_back(tag, p->id(), data);
  return true;
}

bool saveParameters(g2o::AbstractGraph& abstract_graph,
                    const g2o::ParameterContainer& params) {
  bool status = true;
  for (const auto& param : params) {
    status = saveParameter(abstract_graph, param.second.get()) && status;
  }
  return status;
}

// helper to add data to the graph
void addDataToGraphElement(
    HyperGraph::DataContainer& dataContainer,
    const std::vector<AbstractGraph::AbstractData>& data_vector) {
  Factory* factory = Factory::instance();

  HyperGraph::GraphElemBitset elemDataBitset;
  elemDataBitset[HyperGraph::kHgetData] = true;
  for (const auto& abstract_data : data_vector) {
    const std::shared_ptr<HyperGraph::HyperGraphElement> element =
        factory->construct(abstract_data.tag, elemDataBitset);
    if (!element) {
      G2O_WARN("{} could not be constructed as data", abstract_data.tag);
      continue;
    }
    assert(element->elementType() == HyperGraph::kHgetData && "Should be data");
    auto d = std::static_pointer_cast<OptimizableGraph::Data>(element);
    std::stringstream buffer(abstract_data.data);
    const bool r = d->read(buffer);
    if (!r) {
      G2O_ERROR("Error reading data {}", abstract_data.tag);
      continue;
    }
    dataContainer.addUserData(d);
  }
};
}  // namespace

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

bool OptimizableGraph::Edge::setParameterId(int argNum, int paramId) {
  if (static_cast<int>(parameters_.size()) <= argNum) return false;
  if (argNum < 0) return false;
  parameters_[argNum] = nullptr;
  parameterIds_[argNum] = paramId;
  return true;
}

bool OptimizableGraph::Edge::resolveParameters(const OptimizableGraph& graph) {
  assert(parameters_.size() == parameterIds_.size());
  for (decltype(parameters_)::size_type i = 0; i < parameters_.size(); i++) {
    const int index = parameterIds_[i];
    parameters_[i] = graph.parameter(index);
    if (!parameters_[i]) {
      G2O_CRITICAL(
          "Parameter {} with expected Id {} not contained in the graph. Check "
          "setParameterId() calls for the edge.",
          i, index);
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

bool OptimizableGraph::Edge::setMeasurementFromState() { return false; }

OptimizableGraph::OptimizableGraph()
    : graphActions_(static_cast<int>(ActionType::kAtNumElements)) {}

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
        "FATAL, a vertex with (negative) ID {} cannot be inserted into the "
        "graph",
        ov->id());
    assert(0 && "Invalid vertex id");
    return false;
  }
  if (vertex(ov->id())) {
    G2O_WARN(
        "a vertex with ID {} has already been registered with this "
        "graph",
        ov->id());
    return false;
  }
  if (userData) ov->addUserData(userData);
  return HyperGraph::addVertex(v);
}

bool OptimizableGraph::addEdge(const std::shared_ptr<HyperGraph::Edge>& he) {
  auto* e = dynamic_cast<OptimizableGraph::Edge*>(he.get());
  if (!e) return false;

  const bool eresult = HyperGraph::addEdge(he);
  if (!eresult) return false;

  e->internalId_ = nextEdgeId_++;
  if (e->numUndefinedVertices()) return true;
  if (!e->resolveParameters(*this)) {
    G2O_ERROR("FATAL, cannot resolve parameters for edge {}",
              static_cast<void*>(e));
    return false;
  }
  if (!e->resolveCaches()) {
    G2O_ERROR("FATAL, cannot resolve caches for edge {}",
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
    if (!ee->resolveParameters(*this)) {
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

bool OptimizableGraph::load(std::istream& is, io::Format format) {
  Factory* factory = Factory::instance();

  g2o::AbstractGraph abstract_graph;
  bool load_status = abstract_graph.load(is, format);
  if (!load_status) {
    G2O_ERROR("Failed to load graph");
    return false;
  }

  if (!renamedTypesLookup_.empty()) {
    abstract_graph.renameTags(renamedTypesLookup_);
  }

  // Create the parameters of the graph
  HyperGraph::GraphElemBitset elemParamBitset;
  elemParamBitset[HyperGraph::kHgetParameter] = true;
  for (const auto& abstract_param : abstract_graph.parameters()) {
    const std::shared_ptr<HyperGraph::HyperGraphElement> pelement =
        factory->construct(abstract_param.tag, elemParamBitset);
    if (!pelement) {
      G2O_WARN("{} could not be constructed as parameter", abstract_param.tag);
      continue;
    }
    assert(pelement->elementType() == HyperGraph::kHgetParameter &&
           "Should be a param");
    auto p = std::static_pointer_cast<Parameter>(pelement);
    p->setId(abstract_param.id);
    if (!p->setParameterData(abstract_param.value)) {
      G2O_WARN("{} could not set parameter data", abstract_param.tag);
      continue;
    }
    if (!parameters_.addParameter(p)) {
      G2O_ERROR("Parameter of type: {} id: {} already defined",
                abstract_param.tag, abstract_param.id);
    }
  }

  // Create the vertices of the graph
  HyperGraph::GraphElemBitset elemVertexBitset;
  elemVertexBitset[HyperGraph::kHgetVertex] = true;
  for (const auto& abstract_vertex : abstract_graph.vertices()) {
    const std::shared_ptr<HyperGraph::HyperGraphElement> graph_element =
        factory->construct(abstract_vertex.tag, elemVertexBitset);
    if (!graph_element) {
      G2O_WARN("{} could not be constructed as vertex", abstract_vertex.tag);
      continue;
    }
    assert(graph_element->elementType() == HyperGraph::kHgetVertex &&
           "Should be a vertex");
    auto vertex = std::static_pointer_cast<Vertex>(graph_element);
    vertex->setId(abstract_vertex.id);
    vertex->setDimension(abstract_vertex.estimate.size());
    if (!vertex->setEstimateData(abstract_vertex.estimate)) {
      G2O_WARN("{} could not set estimate", abstract_vertex.tag);
      continue;
    }
    if (!addVertex(vertex)) {
      G2O_ERROR("Failure adding Vertex {} {}", abstract_vertex.tag,
                abstract_vertex.id);
    }
    if (!abstract_vertex.data.empty())
      addDataToGraphElement(*vertex, abstract_vertex.data);
  }

  // Create the edges of the graph
  HyperGraph::GraphElemBitset elemEdgeBitset;
  elemEdgeBitset[HyperGraph::kHgetEdge] = true;
  for (const auto& abstract_edge : abstract_graph.edges()) {
    const std::shared_ptr<HyperGraph::HyperGraphElement> graph_element =
        factory->construct(abstract_edge.tag, elemEdgeBitset);
    if (!graph_element) {
      G2O_WARN("{} could not be constructed as edge", abstract_edge.tag);
      continue;
    }
    assert(graph_element->elementType() == HyperGraph::kHgetEdge &&
           "Should be an edge");
    auto edge = std::static_pointer_cast<Edge>(graph_element);
    edge->resize(abstract_edge.ids.size());
    bool vertsOkay = true;
    for (std::vector<int>::size_type l = 0; l < abstract_edge.ids.size(); ++l) {
      const int vertexId = abstract_edge.ids[l];
      if (vertexId != HyperGraph::kUnassignedId) {
        auto v = vertex(vertexId);
        if (!v) {
          vertsOkay = false;
          break;
        }
        edge->setVertex(l, v);
      }
    }
    if (!vertsOkay) {
      G2O_ERROR(
          "Unable to find vertices for edge {} IDs: {}", abstract_edge.tag,
          strJoin(abstract_edge.ids.begin(), abstract_edge.ids.end(), " "));
      continue;
    }
    for (size_t i = 0; i < abstract_edge.param_ids.size(); ++i) {
      edge->setParameterId(i, abstract_edge.param_ids[i]);
    }
    if (!edge->setMeasurementData(abstract_edge.measurement.data())) {
      G2O_WARN("{} could not set measurement", abstract_edge.tag);
      continue;
    }
    MatrixX::MapType information(edge->informationData(), edge->dimension(),
                                 edge->dimension());
    for (int r = 0, idx = 0; r < information.rows(); ++r)
      for (int c = r; c < information.cols(); ++c) {
        information(r, c) = abstract_edge.information[idx++];
        if (r != c) information(c, r) = information(r, c);
      }
    if (!addEdge(edge)) {
      G2O_ERROR(
          "Failure adding Edge {} IDs {}", abstract_edge.tag,
          strJoin(abstract_edge.ids.begin(), abstract_edge.ids.end(), " "));
    }
    if (!abstract_edge.data.empty())
      addDataToGraphElement(*edge, abstract_edge.data);
  }

  for (const auto fixed_vertex_id : abstract_graph.fixed()) {
    auto v = vertex(fixed_vertex_id);
    if (!v) {
      G2O_WARN("Cannot fix vertex {}", fixed_vertex_id);
      continue;
    }
    v->setFixed(true);
  }

  G2O_TRACE("Loaded {} parameters", parameters_.size());
  G2O_TRACE("Loaded {} vertices", vertices_.size());
  G2O_TRACE("Loaded {} edges", edges_.size());

  return true;
}

bool OptimizableGraph::load(const char* filename, io::Format format) {
  std::ifstream ifs(filename, format == io::Format::kBinary
                                  ? std::ios_base::in | std::ios::binary
                                  : std::ios_base::in);
  if (!ifs) {
    G2O_ERROR("Unable to open file {}", filename);
    return false;
  }
  return load(ifs, format);
}

bool OptimizableGraph::save(const char* filename, io::Format format,
                            int level) const {
  std::ofstream ofs(filename, format == io::Format::kBinary
                                  ? std::ios_base::out | std::ios::binary
                                  : std::ios_base::out);
  if (!ofs) return false;
  return save(ofs, format, level);
}

bool OptimizableGraph::save(std::ostream& os, io::Format format,
                            int level) const {
  g2o::AbstractGraph abstract_graph;
  bool status = saveParameters(abstract_graph, parameters_);

  // write the parameters to the top of the file
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

  for (auto* v : verticesToSave) status &= saveVertex(abstract_graph, v);

  std::vector<std::shared_ptr<HyperGraph::Edge>> edgesToSave;
  std::copy_if(edges().begin(), edges().end(), std::back_inserter(edgesToSave),
               [level](const std::shared_ptr<HyperGraph::Edge>& ee) {
                 auto* e = dynamic_cast<OptimizableGraph::Edge*>(ee.get());
                 return (e->level() == level);
               });
  sort(edgesToSave.begin(), edgesToSave.end(), EdgeIDCompare());
  for (const auto& e : edgesToSave)
    status &= saveEdge(abstract_graph, static_cast<Edge*>(e.get()));

  return abstract_graph.save(os, format) && status;
}

bool OptimizableGraph::saveSubset(std::ostream& os, HyperGraph::VertexSet& vset,
                                  io::Format format, int level) {
  g2o::AbstractGraph abstract_graph;
  bool status = saveParameters(abstract_graph, parameters_);

  for (const auto& v : vset)
    saveVertex(abstract_graph, static_cast<Vertex*>(v.get()));

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
    status &= saveEdge(abstract_graph, e);
  }
  return abstract_graph.save(os, format) && status;
}

bool OptimizableGraph::saveSubset(std::ostream& os, HyperGraph::EdgeSet& eset,
                                  io::Format format) {
  g2o::AbstractGraph abstract_graph;
  bool status = saveParameters(abstract_graph, parameters_);
  HyperGraph::VertexSet vset;
  for (const auto& e : eset)
    for (const auto& v : e->vertices())
      if (v) vset.insert(v);

  for (const auto& v : vset)
    status &= saveVertex(abstract_graph, static_cast<Vertex*>(v.get()));
  for (const auto& e : eset)
    status &= saveEdge(abstract_graph, static_cast<Edge*>(e.get()));
  return abstract_graph.save(os, format) && status;
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
  const std::vector<std::string> typesMap = strSplit(types, ",");
  for (const auto& i : typesMap) {
    const std::vector<std::string> m = strSplit(i, "=");
    if (m.size() != 2) {
      G2O_ERROR("unable to extract type map from {}", i);
      continue;
    }
    const std::string typeInFile = trim(m[0]);
    const std::string loadedType = trim(m[1]);
    if (!factory->knowsTag(loadedType)) {
      G2O_ERROR("unknown type {}", loadedType);
      continue;
    }

    renamedTypesLookup_[typeInFile] = loadedType;
  }

  G2O_DEBUG("Load look up table:");
  for (const auto& rtl : renamedTypesLookup_) {
    G2O_DEBUG("{} -> {}", rtl.first, rtl.second);
  }
}

bool OptimizableGraph::isSolverSuitable(
    const OptimizationAlgorithmProperty& solverProperty,
    const std::unordered_set<int>& vertDims_) const {
  const std::unordered_set<int>& vertDims =
      vertDims_.empty() ? dimensions() : vertDims_;
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

std::unordered_set<int> OptimizableGraph::dimensions() const {
  std::unordered_set<int> result;
  for (const auto& id_v : vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(id_v.second.get());
    result.insert(v->dimension());
  }
  return result;
}

void OptimizableGraph::performActions(int iter, HyperGraphActionSet& actions) {
  if (actions.empty()) return;
  HyperGraphAction::ParametersIteration params(iter);
  for (const auto& action : actions) {
    (*action)(*this, params);
  }
}

void OptimizableGraph::preIteration(int iter) {
  performActions(iter,
                 graphActions_[static_cast<int>(ActionType::kAtPreiteration)]);
}

void OptimizableGraph::postIteration(int iter) {
  performActions(iter,
                 graphActions_[static_cast<int>(ActionType::kAtPostiteration)]);
}

bool OptimizableGraph::addPostIterationAction(
    std::shared_ptr<HyperGraphAction> action) {
  const std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      graphActions_[static_cast<int>(ActionType::kAtPostiteration)].emplace(
          std::move(action));
  return insertResult.second;
}

bool OptimizableGraph::addPreIterationAction(
    std::shared_ptr<HyperGraphAction> action) {
  const std::pair<HyperGraphActionSet::iterator, bool> insertResult =
      graphActions_[static_cast<int>(ActionType::kAtPreiteration)].emplace(
          std::move(action));
  return insertResult.second;
}

bool OptimizableGraph::removePreIterationAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  return graphActions_[static_cast<int>(ActionType::kAtPreiteration)].erase(
             action) > 0;
}

bool OptimizableGraph::removePostIterationAction(
    const std::shared_ptr<HyperGraphAction>& action) {
  return graphActions_[static_cast<int>(ActionType::kAtPostiteration)].erase(
             action) > 0;
}

bool OptimizableGraph::saveVertex(AbstractGraph& abstract_graph,
                                  OptimizableGraph::Vertex* v) {
  Factory* factory = Factory::instance();
  const std::string tag = factory->tag(v);
  if (tag.empty()) {
    G2O_WARN("Got empty tag for vertex {} while saving", v->id());
    return false;
  }
  std::vector<double> vertex_estimate;
  v->getEstimateData(vertex_estimate);
  abstract_graph.vertices().emplace_back(tag, v->id(), vertex_estimate);
  saveUserData(abstract_graph.vertices().back(), v->userData());
  if (v->fixed()) {
    abstract_graph.fixed().push_back(v->id());
  }
  return true;
}

bool OptimizableGraph::saveEdge(AbstractGraph& abstract_graph,
                                OptimizableGraph::Edge* e) {
  Factory* factory = Factory::instance();
  const std::string tag = factory->tag(e);
  if (tag.empty()) {
    G2O_WARN("Got empty tag for edge while saving");
    return false;
  }
  std::vector<int> ids;
  ids.reserve(e->vertices().size());
  for (const auto& vertex : e->vertices()) {
    ids.push_back(vertex ? vertex->id() : HyperGraph::kUnassignedId);
  }
  std::vector<double> data(e->measurementDimension());
  e->getMeasurementData(data.data());
  MatrixX::MapType information(e->informationData(), e->dimension(),
                               e->dimension());
  std::vector<double> upper_triangle;
  upper_triangle.reserve((e->dimension() * (e->dimension() + 1)) / 2);
  for (int r = 0; r < e->dimension(); ++r)
    for (int c = r; c < e->dimension(); ++c)
      upper_triangle.push_back(information(r, c));
  abstract_graph.edges().emplace_back(tag, ids, data, upper_triangle,
                                      e->parameterIds());
  saveUserData(abstract_graph.edges().back(), e->userData());
  return true;
}

void OptimizableGraph::clear() { HyperGraph::clear(); }

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
        std::vector<int> ids;
        ids.reserve(e->vertices().size());
        for (const auto& v : e->vertices()) ids.push_back(v->id());
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

void OptimizableGraph::addGraph(OptimizableGraph& other) {
  jacobianWorkspace_.updateSize(other.jacobianWorkspace());
  jacobianWorkspace_.allocate();

  // parameters
  for (const auto& p : other.parameters_) {
    parameters_.emplace(p.first, p.second);
  }
  other.parameters_.clear();

  // vertices
  for (const auto& id_v : other.vertices_) {
    vertices_.emplace(id_v.first, id_v.second);
  }
  other.vertices_.clear();

  // edges
  for (const auto& e : other.edges_) {
    edges_.emplace(e);
  }
  other.edges_.clear();
}

bool OptimizableGraph::initMultiThreading() {
#if (defined G2O_OPENMP) && EIGEN_VERSION_AT_LEAST(3, 1, 0)
  Eigen::initParallel();
#endif
  return true;
}

}  // namespace g2o
