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

#include <iostream>
#include <iomanip>
#include <csignal>
#include <cstdlib>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "slam_parser/interface/parser_interface.h"

#include "g2o_slam_interface.h"
#include "graph_optimizer_sparse_online.h"

static bool hasToStop=false;

using namespace std;
using namespace g2o;

void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

int main(int argc, char** argv)
{
  bool pcg;
  int updateEachN;
  bool vis;
  bool verbose;
  // command line parsing
  CommandArgs arg;
  arg.param("update", updateEachN, 10, "update the graph after inserting N nodes");
  arg.param("pcg", pcg, false, "use PCG instead of Cholesky");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("g", vis, false, "gnuplot visualization");

  arg.parseArgs(argc, argv);

  SparseOptimizerOnline optimizer(pcg);
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);
  optimizer.vizWithGnuplot = vis;

  G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);

  SlamParser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(cin))
  {
    // do something additional if needed
  }

  return 0;
}
