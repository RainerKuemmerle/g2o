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
  //SparseOptimizer optimizer;
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
