#include "slam_context.h"

#include "commands.h"

#include <iostream>
using namespace std;

namespace SlamParser {

SlamContext::SlamContext()
{
}

SlamContext::~SlamContext()
{
}

bool SlamContext::process(CommandNode* commandNode)
{
  cerr << "SlamContext::process -> ";
  switch (commandNode->commandType()) {
    case CT_ADD_NODE:
      cerr << "ADD NODE" << endl;
      break;
    case CT_ADD_EDGE:
      cerr << "ADD EDGE" << endl;
      break;
    case CT_SOLVE_STATE:
      cerr << "SOLVE STATE" << endl;
      break;
    case CT_QUERY_STATE:
      cerr << "QUERY STATE" << endl;
      break;
    case CT_FIX:
      cerr << "FIX NODE" << endl;
      break;
  }
  return true;
}

} // end namespace
