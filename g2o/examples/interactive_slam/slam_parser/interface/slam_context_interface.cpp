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

#include "slam_context_interface.h"

#include <iostream>

#include "abstract_slam_interface.h"
#include "slam_parser/parser/commands.h"

namespace slam_parser {

SlamContextInterface::SlamContextInterface(AbstractSlamInterface* slam)
    : slam_(slam) {}

bool SlamContextInterface::process(CommandNode* commandNode) {
  if (!slam_) {
    return true;
  }
  switch (commandNode->commandType()) {
    case kCtAddNode: {
      auto* c = static_cast<AddNode*>(commandNode);
      return slam_->addNode(c->tag(), c->id(), c->dimension(), c->values());
    }
    case kCtAddEdge: {
      auto* c = static_cast<AddEdge*>(commandNode);
      return slam_->addEdge(c->tag(), c->id(), c->dimension(), c->id1(),
                            c->id2(), c->values(), c->information());
    }
    case kCtSolveState: {
      // SolveState* c = static_cast<SolveState*>(commandNode);
      return slam_->solveState();
    }
    case kCtQueryState: {
      auto* c = static_cast<QueryState*>(commandNode);
      return slam_->queryState(c->ids());
    }
    case kCtFix: {
      auto* c = static_cast<FixNode*>(commandNode);
      return slam_->fixNode(c->ids());
    }
  }
  std::cerr << "SlamContextInterface::process: Unknown command type"
            << std::endl;
  return false;
}

}  // namespace slam_parser
