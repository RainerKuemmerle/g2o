// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <cassert>
#include <csignal>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"

#include "slam_parser/interface/parser_interface.h"

#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"
#include "graph_optimizer_sparse_incremental.h"

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
  string outputFilename;
  int updateEachN;
  bool verbose;
  bool vis;
  // command line parsing
  CommandArgs arg;
  arg.param("update", updateEachN, 10, "update the graph after inserting N nodes");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("g", vis, false, "gnuplot visualization");
  arg.param("o", outputFilename, "", "output the final graph");

  arg.parseArgs(argc, argv);

  SparseOptimizerIncremental optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);
  optimizer.vizWithGnuplot = vis;

  G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);

  cerr << "Updating every " << updateEachN << endl;

  SlamParser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(cin)) {}

  if (outputFilename.size() > 0) {
    cerr << "Saving " << outputFilename << endl;
    optimizer.save(outputFilename.c_str());
  }

  return 0;
}
