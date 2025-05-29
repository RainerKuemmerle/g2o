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

#include <csignal>
#include <cstdlib>

#include "CLI/CLI.hpp"
#include "g2o_slam_interface.h"
#include "graph_optimizer_sparse_online.h"
#include "slam_parser/interface/parser_interface.h"

int main(int argc, char** argv) {
  bool pcg;
  int updateEachN = 10;
  bool vis;
  bool verbose;
  // command line parsing
  CLI::App app{"g2o Online"};
  argv = app.ensure_utf8(argv);
  app.add_option("--update", updateEachN,
                 "update the graph after inserting N nodes")
      ->check(CLI::PositiveNumber);
  app.add_flag("--pcg", pcg, "use PCG instead of Cholesky");
  app.add_flag("-v", verbose, "verbose output of the optimization process");
  app.add_flag("-g", vis, "gnuplot visualization");

  CLI11_PARSE(app, argc, argv);

  g2o::SparseOptimizerOnline optimizer(pcg);
  optimizer.setVerbose(verbose);
  optimizer.vizWithGnuplot = vis;

  g2o::G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);

  slam_parser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(std::cin)) {
    // do something additional if needed
  }

  return 0;
}
