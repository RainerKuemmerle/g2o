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

#ifndef G2O_HYPER_GRAPH_ACTION_H
#define G2O_HYPER_GRAPH_ACTION_H

#include <map>
#include <memory>
#include <string>

#include "g2o/stuff/property.h"
#include "g2o_core_api.h"
#include "hyper_graph.h"

// define to get verbose output
// #define G2O_DEBUG_ACTIONLIB

namespace g2o {

class CacheContainer;

/**
 * \brief Abstract action that operates on an entire graph
 */
class G2O_CORE_API HyperGraphAction {
 public:
  class G2O_CORE_API Parameters {
   public:
    virtual ~Parameters();
  };

  class G2O_CORE_API ParametersIteration : public Parameters {
   public:
    explicit ParametersIteration(int iter);
    int iteration;
  };

  virtual ~HyperGraphAction();

  /**
   * re-implement to carry out an action given the graph
   */
  virtual bool operator()(
      const HyperGraph& graph,
      const std::shared_ptr<Parameters>& parameters = nullptr);
};

/**
 * \brief Abstract action that operates on a graph entity
 */
class G2O_CORE_API HyperGraphElementAction {
 public:
  struct G2O_CORE_API Parameters {
    virtual ~Parameters();
  };
  using HyperGraphElementActionPtr = std::shared_ptr<HyperGraphElementAction>;
  //! an action should be instantiated with the typeid.name of the graph element
  //! on which it operates
  explicit HyperGraphElementAction(std::string typeName = "");

  //! redefine this to do the action stuff. If successful, returns true.
  virtual bool operator()(HyperGraph::HyperGraphElement& element,
                          const std::shared_ptr<Parameters>& parameters);

  virtual ~HyperGraphElementAction();

  //! returns the typeid name of the action
  [[nodiscard]] const std::string& typeName() const { return typeName_; }

  //! returns the name of an action, e.g "draw"
  [[nodiscard]] const std::string& name() const { return name_; }

  //! sets the type on which an action has to operate
  void setTypeName(std::string typeName);

 protected:
  std::string typeName_;
  std::string name_;
};

/**
 * \brief collection of actions
 *
 * collection of actions calls contains homogeneous actions operating on
 * different types all collected actions have the same name and should have the
 * same functionality
 */
class G2O_CORE_API HyperGraphElementActionCollection
    : public HyperGraphElementAction {
 public:
  using ActionMap = std::map<std::string, HyperGraphElementActionPtr>;
  //! constructor. name_ is the name of the action e.g.draw).
  explicit HyperGraphElementActionCollection(const std::string& name_);
  /**
   * calling functions, they return a pointer to the instance of action in
   * actionMap that was active on element
   */
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<Parameters>& parameters) override;
  ActionMap& actionMap() { return actionMap_; }
  //! inserts an action in the pool. The action should have the same name of the
  //! container. returns false on failure (the container has a different name
  //! than the action);
  bool registerAction(
      const HyperGraphElementAction::HyperGraphElementActionPtr& action);
  bool unregisterAction(
      const HyperGraphElementAction::HyperGraphElementActionPtr& action);

 protected:
  ActionMap actionMap_;
};

/**
 * \brief library of actions, indexed by the action name;
 *
 * library of actions, indexed by the action name;
 * one can use ti to register a collection of actions
 */
class G2O_CORE_API HyperGraphActionLibrary {
 public:
  //! return the single instance of the HyperGraphActionLibrary
  static HyperGraphActionLibrary* instance();
  //! free the instance
  static void destroy();

  HyperGraphActionLibrary(HyperGraphActionLibrary const&) = delete;
  HyperGraphActionLibrary& operator=(HyperGraphActionLibrary const&) = delete;

  // returns a pointer to a collection indexed by name
  HyperGraphElementAction::HyperGraphElementActionPtr actionByName(
      const std::string& name);
  // registers a basic action in the pool. If necessary a container is created
  bool registerAction(
      const HyperGraphElementAction::HyperGraphElementActionPtr& action);
  bool unregisterAction(
      const HyperGraphElementAction::HyperGraphElementActionPtr& action);

  HyperGraphElementActionCollection::ActionMap& actionMap() {
    return actionMap_;
  }

 protected:
  HyperGraphActionLibrary() = default;
  HyperGraphElementActionCollection::ActionMap actionMap_;

 private:
  static std::unique_ptr<HyperGraphActionLibrary> actionLibInstance_;
};

/**
 * apply an action to all the elements of the graph.
 */
void G2O_CORE_API applyAction(
    HyperGraph& graph, HyperGraphElementAction& action,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& parameters =
        nullptr,
    const std::string& typeName = "");

/**
 * \brief draw actions
 */
class G2O_CORE_API DrawAction : public HyperGraphElementAction {
 public:
  class G2O_CORE_API Parameters : public HyperGraphElementAction::Parameters,
                                  public PropertyMap {
   public:
    Parameters();
  };
  explicit DrawAction(const std::string& typeName_);

 protected:
  virtual bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params);
  void initializeDrawActionsCache();
  void drawCache(
      const CacheContainer& caches,
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params);
  void drawUserData(
      const HyperGraph::DataContainer::DataVector& data,
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params);
  std::shared_ptr<Parameters> previousParams_;
  std::shared_ptr<BoolProperty> show_;
  HyperGraphElementAction::HyperGraphElementActionPtr cacheDrawActions_;
};

template <typename T>
class RegisterActionProxy {
 public:
  RegisterActionProxy() {
#ifdef G2O_DEBUG_ACTIONLIB
    std::cout << __FUNCTION__ << ": Registering action of type "
              << typeid(T).name() << std::endl;
#endif
    action_.reset(new T());
    HyperGraphActionLibrary::instance()->registerAction(action_);
  }

 private:
  HyperGraphElementAction::HyperGraphElementActionPtr action_;
};

}  // namespace g2o

#define G2O_REGISTER_ACTION(classname)                            \
  extern "C" void g2o_action_##classname(void) {}                 \
  namespace {                                                     \
  g2o::RegisterActionProxy<classname> g_action_proxy_##classname; \
  }

#endif
