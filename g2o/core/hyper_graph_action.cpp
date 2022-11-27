// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "hyper_graph_action.h"

#include <iostream>
#include <list>
#include <utility>

#include "cache.h"
#include "g2o/stuff/macros.h"
#include "optimizable_graph.h"

namespace g2o {

std::unique_ptr<HyperGraphActionLibrary>
    HyperGraphActionLibrary::actionLibInstance_;

HyperGraphAction::Parameters::~Parameters() = default;

HyperGraphAction::ParametersIteration::ParametersIteration(int iter)
    : HyperGraphAction::Parameters(), iteration(iter) {}

HyperGraphAction::~HyperGraphAction() = default;

bool HyperGraphAction::operator()(const HyperGraph&,
                                  const std::shared_ptr<Parameters>&) {
  return false;
}

HyperGraphElementAction::Parameters::~Parameters() = default;

HyperGraphElementAction::HyperGraphElementAction(std::string typeName)
    : typeName_(std::move(typeName)) {}

void HyperGraphElementAction::setTypeName(std::string typeName) {
  typeName_ = std::move(typeName);
}

bool HyperGraphElementAction::operator()(HyperGraph::HyperGraphElement&,
                                         const std::shared_ptr<Parameters>&) {
  return false;
}

HyperGraphElementAction::~HyperGraphElementAction() = default;

HyperGraphElementActionCollection::HyperGraphElementActionCollection(
    const std::string& name) {
  name_ = name;
}

bool HyperGraphElementActionCollection::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<Parameters>& parameters) {
  auto it = actionMap_.find(typeid(element).name());
  if (it == actionMap_.end()) return false;
  HyperGraphElementAction* action = it->second.get();
  return (*action)(element, parameters);
}

bool HyperGraphElementActionCollection::registerAction(
    const HyperGraphElementAction::HyperGraphElementActionPtr& action) {
#ifdef G2O_DEBUG_ACTIONLIB
  cerr << __PRETTY_FUNCTION__ << " " << action->name() << " "
       << action->typeName() << endl;
#endif
  if (action->name() != name()) {
    std::cerr << __PRETTY_FUNCTION__
              << ": invalid attempt to register an action in a collection with "
                 "a different name "
              << name() << " " << action->name() << std::endl;
  }
  actionMap_.insert(make_pair(action->typeName(), action));
  return true;
}

bool HyperGraphElementActionCollection::unregisterAction(
    const HyperGraphElementAction::HyperGraphElementActionPtr& action) {
  for (auto it = actionMap_.begin(); it != actionMap_.end(); ++it) {
    if (it->second == action) {
      actionMap_.erase(it);
      return true;
    }
  }
  return false;
}

HyperGraphActionLibrary* HyperGraphActionLibrary::instance() {
  if (actionLibInstance_ == nullptr) {
    actionLibInstance_ =
        std::unique_ptr<HyperGraphActionLibrary>(new HyperGraphActionLibrary);
  }
  return actionLibInstance_.get();
}

void HyperGraphActionLibrary::destroy() {
  std::unique_ptr<HyperGraphActionLibrary> aux;
  actionLibInstance_.swap(aux);
}

HyperGraphElementAction::HyperGraphElementActionPtr
HyperGraphActionLibrary::actionByName(const std::string& name) {
  auto it = actionMap_.find(name);
  if (it != actionMap_.end()) return it->second;
  return nullptr;
}

bool HyperGraphActionLibrary::registerAction(
    const HyperGraphElementAction::HyperGraphElementActionPtr& action) {
  const HyperGraphElementAction::HyperGraphElementActionPtr oldAction =
      actionByName(action->name());
  std::shared_ptr<HyperGraphElementActionCollection> collection;
  if (oldAction) {
    collection =
        std::dynamic_pointer_cast<HyperGraphElementActionCollection>(oldAction);
    if (!collection) {
      std::cerr << __PRETTY_FUNCTION__
                << ": fatal error, a collection is not at the first level in "
                   "the library"
                << std::endl;
      return false;
    }
    return collection->registerAction(action);
  }
#ifdef G2O_DEBUG_ACTIONLIB
  cerr << __PRETTY_FUNCTION__ << ": creating collection for \""
       << action->name() << "\"" << endl;
#endif
  collection =
      std::make_shared<HyperGraphElementActionCollection>(action->name());
  actionMap_.insert(std::make_pair(action->name(), collection));
  return collection->registerAction(action);
}

bool HyperGraphActionLibrary::unregisterAction(
    const HyperGraphElementAction::HyperGraphElementActionPtr& action) {
  std::list<HyperGraphElementActionCollection*> collectionDeleteList;

  // Search all the collections and delete the registered actions; if a
  // collection becomes empty, schedule it for deletion; note that we can't
  // delete the collections as we go because this will screw up the state of the
  // iterators
  for (auto& it : actionMap_) {
    auto* collection =
        dynamic_cast<HyperGraphElementActionCollection*>(it.second.get());
    if (collection != nullptr) {
      collection->unregisterAction(action);
      if (collection->actionMap().empty()) {
        collectionDeleteList.push_back(collection);
      }
    }
  }

  // Delete any empty action collections
  for (auto& itc : collectionDeleteList) {
    // cout << "Deleting collection " << (*itc)->name() << endl;
    actionMap_.erase(itc->name());
  }

  return true;
}

WriteGnuplotAction::WriteGnuplotAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_) {
  name_ = "writeGnuplot";
}

DrawAction::Parameters::Parameters() = default;

DrawAction::DrawAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_) {
  name_ = "draw";
  // previousParams_.reset(reinterpret_cast<Parameters*>(0x42));
  refreshPropertyPtrs(nullptr);
  cacheDrawActions_ = nullptr;
}

bool DrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (previousParams_ == params) return false;
  auto p = std::dynamic_pointer_cast<DrawAction::Parameters>(params);
  if (!p) {
    previousParams_ = nullptr;
    show_ = nullptr;
    showId_ = nullptr;
  } else {
    previousParams_ = p;
    show_ = p->makeProperty<BoolProperty>(typeName_ + "::SHOW", true);
    showId_ = p->makeProperty<BoolProperty>(typeName_ + "::SHOW_ID", false);
  }
  return true;
}

void DrawAction::initializeDrawActionsCache() {
  if (!cacheDrawActions_) {
    cacheDrawActions_ =
        HyperGraphActionLibrary::instance()->actionByName("draw");
  }
}

void DrawAction::drawCache(
    const std::shared_ptr<CacheContainer>& caches,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (caches) {
    for (auto& cache : *caches) {
      Cache* c = cache.second.get();
      (*cacheDrawActions_)(*c, params);
    }
  }
}

void DrawAction::drawUserData(
    const std::shared_ptr<HyperGraph::Data>& data,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  HyperGraph::Data* dataPtr = data.get();
  while (data && cacheDrawActions_) {
    (*cacheDrawActions_)(*dataPtr, params);
    dataPtr = dataPtr->next().get();
  }
}

void applyAction(
    HyperGraph& graph, HyperGraphElementAction& action,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& parameters,
    const std::string& typeName) {
  for (auto& it : graph.vertices()) {
    auto& aux = *it.second;
    if (typeName.empty() || typeid(aux).name() == typeName) {
      (action)(*it.second, parameters);
    }
  }
  for (const auto& it : graph.edges()) {
    auto& aux = *it;
    if (typeName.empty() || typeid(aux).name() == typeName)
      (action)(aux, parameters);
  }
}

}  // namespace g2o
