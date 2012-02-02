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

#ifndef G2O_OUTPUT_HELPER_H
#define G2O_OUTPUT_HELPER_H

#include <string>
#include <iosfwd>
#include "g2o/core/hyper_graph.h"
#include "g2o_cli_api.h"

namespace g2o {

  struct OptimizableGraph;

  /**
   * save the state of the optimizer into files for visualizing using Gnuplot
   */
  G2O_CLI_API bool saveGnuplot(const std::string& gnudump, const OptimizableGraph& optimizer);
  G2O_CLI_API bool saveGnuplot(const std::string& gnudump, const HyperGraph::VertexSet& vertices, const HyperGraph::EdgeSet& edges);

  /**
   * dump the edges to a stream, e.g., cout and redirect to gnuplot
   */
  G2O_CLI_API bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer);

} // end namespace

#endif
