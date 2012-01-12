#include "slam_context_interface.h"

#include "abstract_slam_interface.h"
#include "slam_parser/parser/commands.h"

#include <iostream>
using namespace std;

namespace SlamParser {

  SlamContextInterface::SlamContextInterface(AbstractSlamInterface* slam) :
    SlamContext(),
    _slam(slam)
  {
  }

  SlamContextInterface::~SlamContextInterface()
  {
  }

  bool SlamContextInterface::process(CommandNode* commandNode)
  {
    if (! _slam) {
      return true;
    }
    switch (commandNode->commandType()) {
      case CT_ADD_NODE:
        {
          AddNode* c = static_cast<AddNode*>(commandNode);
          return _slam->addNode(c->tag(), c->id(), c->dimension(), c->values());
        }
      case CT_ADD_EDGE:
        {
          AddEdge* c = static_cast<AddEdge*>(commandNode);
          return _slam->addEdge(c->tag(), c->id(), c->dimension(), c->id1(), c->id2(), c->values(), c->information());
        }
      case CT_SOLVE_STATE:
        {
          //SolveState* c = static_cast<SolveState*>(commandNode);
          return _slam->solveState();
        }
      case CT_QUERY_STATE:
        {
          QueryState* c = static_cast<QueryState*>(commandNode);
          return _slam->queryState(c->ids());
        }
      case CT_FIX:
        {
          FixNode* c = static_cast<FixNode*>(commandNode);
          return _slam->fixNode(c->ids());
        }
    }
    cerr << "SlamContextInterface::process: Unknown command type" << endl;
    return false;
  }

} // end namespace
