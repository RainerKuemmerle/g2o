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

#ifndef G2O_HYPER_GRAPH_ACTION_H
#define G2O_HYPER_GRAPH_ACTION_H

#include "hyper_graph.h"
#include "g2o/stuff/property.h"

#include <typeinfo>
#include <iosfwd>
#include <set>
#include <string>

#include "g2o_core_api.h"

namespace g2o {

  /**
   * \brief Abstract action that operates on an entire graph
   */
  class G2O_CORE_API HyperGraphAction {
    public:
      class Parameters {
        public:
          virtual ~Parameters();
      };

      class ParametersIteration : public Parameters {
        public:
          explicit ParametersIteration(int iter);
          int iteration;
      };

      virtual ~HyperGraphAction();

      /**
       * re-implement to carry out an action given the graph
       */
      virtual HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters = 0);
  };

  /**
   * \brief Abstract action that operates on a graph entity
   */
  class G2O_CORE_API HyperGraphElementAction{
    public:
      struct Parameters{
        virtual ~Parameters();
      };
      typedef std::map<std::string, HyperGraphElementAction*> ActionMap;
      //! an action should be instantiated with the typeid.name of the graph element 
      //! on which it operates
      HyperGraphElementAction(const std::string& typeName_="");

      //! redefine this to do the action stuff. If successful, the action returns a pointer to itself
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, Parameters* parameters);

      //! redefine this to do the action stuff. If successful, the action returns a pointer to itself
      virtual HyperGraphElementAction* operator()(const HyperGraph::HyperGraphElement* element, Parameters* parameters);

      //! destroyed actions release the memory
      virtual ~HyperGraphElementAction();

      //! returns the typeid name of the action
      const std::string& typeName() const { return _typeName;}

      //! returns the name of an action, e.g "draw"
      const std::string& name() const{ return _name;}

      //! sets the type on which an action has to operate
      void setTypeName(const std::string& typeName_);

    protected:
      std::string _typeName;
      std::string _name;
  };

  /**
   * \brief collection of actions
   *
   * collection of actions calls contains homogeneous actions operating on different types
   * all collected actions have the same name and should have the same functionality
   */
  class G2O_CORE_API HyperGraphElementActionCollection: public HyperGraphElementAction{
    public:
      //! constructor. name_ is the name of the action e.g.draw).
      HyperGraphElementActionCollection(const std::string& name_);
      //! destructor: it deletes all actions in the pool.
      virtual ~HyperGraphElementActionCollection();
      //! calling functions, they return a pointer to the instance of action in actionMap
      //! that was active on element
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, Parameters* parameters);
      virtual HyperGraphElementAction* operator()(const HyperGraph::HyperGraphElement* element, Parameters* parameters);
      ActionMap& actionMap() {return _actionMap;}
      //! inserts an action in the pool. The action should have the same name of the container.
      //! returns falseon failure (the container has a different name than the action);
      bool registerAction(HyperGraphElementAction* action);
    protected:
      ActionMap _actionMap;
  };

  /**
   * \brief library of actions, indexed by the action name;
   *
   * library of actions, indexed by the action name;
   * one can use ti to register a collection of actions
   */
  class G2O_CORE_API HyperGraphActionLibrary{
    public:
      //! return the single instance of the HyperGraphActionLibrary
      static HyperGraphActionLibrary* instance();
      //! free the instance
      static void destroy();

      // returns a pointer to a collection indexed by name
      HyperGraphElementAction* actionByName(const std::string& name);
      // registers a basic action in the pool. If necessary a container is created
      bool registerAction(HyperGraphElementAction* action);
      inline HyperGraphElementAction::ActionMap& actionMap() {return _actionMap;}
    protected:
      HyperGraphActionLibrary();
      ~HyperGraphActionLibrary();
      HyperGraphElementAction::ActionMap _actionMap;
    private:
      static HyperGraphActionLibrary* actionLibInstance;
  };

  /**
   * apply an action to all the elements of the graph.
   */
  void G2O_CORE_API applyAction(HyperGraph* graph, HyperGraphElementAction* action, HyperGraphElementAction::Parameters* parameters=0, const std::string& typeName="");

  /**
   * brief write into gnuplot
   */
  class G2O_CORE_API WriteGnuplotAction: public HyperGraphElementAction{
    public:
      struct Parameters: public HyperGraphElementAction::Parameters{
        std::ostream* os;
      };
      WriteGnuplotAction(const std::string& typeName_);
  };

  /**
   * \brief draw actions
   */

  class G2O_CORE_API DrawAction : public HyperGraphElementAction{
  public:
    class Parameters: public HyperGraphElementAction::Parameters,  public PropertyMap{
    public:
      Parameters();
    };
    DrawAction(const std::string& typeName_);
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    Parameters* _previousParams;
    BoolProperty* _show;
    BoolProperty* _showId;
  };
  
};

#endif
