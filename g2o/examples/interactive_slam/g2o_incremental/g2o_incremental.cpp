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
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/tictoc.h"

#include "slam_parser/interface/parser_interface.h"

#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"
#include "graph_optimizer_sparse_incremental.h"

static bool hasToStop=false;

using namespace std;
using namespace g2o;

/**
 * \brief Store the information parsed from a g2o file
 */
struct EdgeInformation
{
  int fromId;
  int toId;
  std::vector<double> measurement;
  std::vector<double> information;
};

/**
 * \brief Sort Edges for inserting them sequentially
 */
struct IncrementalEdgesCompare
{
  bool operator()(const EdgeInformation& e1, const EdgeInformation& e2)
  {
    int i11 = e1.fromId, i12 = e1.toId;
    if (i11 > i12)
      swap(i11, i12);
    int i21 = e2.fromId, i22 = e2.toId;
    if (i21 > i22)
      swap(i21, i22);
    if (i12 < i22)
      return true;
    if (i12 > i22)
      return false;
    return i11 < i21;
  }
};

inline void solveAndPrint(G2oSlamInterface& slamInterface, bool verbose)
{
  G2oSlamInterface::SolveResult solverState = slamInterface.solve();
  if (!verbose) {
    switch (solverState) {
      case G2oSlamInterface::SOLVED:
        cout << "."; //<< flush;
        break;
      case G2oSlamInterface::SOLVED_BATCH:
        cout << "b " << slamInterface.optimizer()->vertices().size() << endl;
        break;
      default:
        break;
    }
  }
}

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
  string inputFilename;
  string outputFilename;
  int updateEachN;
  int batchEachN;
  bool verbose;
  bool vis;
  // command line parsing
  CommandArgs arg;
  arg.param("batch", batchEachN, 100, "solve by a batch Cholesky after inserting N nodes");
  arg.param("update", updateEachN, 10, "update the graph after inserting N nodes");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("g", vis, false, "gnuplot visualization");
  arg.param("o", outputFilename, "", "output the final graph");
  arg.param("i", inputFilename, "", "input file (default g2o format), if not given read via stdin");

  arg.parseArgs(argc, argv);

  SparseOptimizerIncremental optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);
  optimizer.vizWithGnuplot = vis;

  G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);
  slamInterface.setBatchSolveEachN(batchEachN);

  cerr << "Updating every " << updateEachN << endl;
  cerr << "Batch step every " << batchEachN << endl;

  if (inputFilename.size() > 0) { // operating on a file
    vector<EdgeInformation> edgesFromGraph;

    // HACK force tictoc statistics
#ifdef _MSC_VER
    _putenv_s("G2O_ENABLE_TICTOC", "1");
#else
    setenv("G2O_ENABLE_TICTOC", "1", 1);
#endif

    // parse the edge from the file
    int graphDimension = 0;
    cerr << "Parsing " << inputFilename << " ... ";
    tictoc("parsing");
    ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      cerr << "Failure to open " << inputFilename << endl;
      return 1;
    }
    stringstream currentLine;
    while (readLine(ifs, currentLine)) {
      string token;
      currentLine >> token;
      if (token == "EDGE_SE2") {
        graphDimension = 3;
        edgesFromGraph.push_back(EdgeInformation());
        EdgeInformation& currentEdge = edgesFromGraph.back();
        currentLine >> currentEdge.fromId >> currentEdge.toId;
        currentEdge.measurement.resize(3);
        currentLine >> currentEdge.measurement[0] >> currentEdge.measurement[1] >> currentEdge.measurement[2];
        currentEdge.information.resize(6);
        for (int i = 0; i < 6; ++i)
          currentLine >> currentEdge.information[i];
      } else if (token == "EDGE_SE3:QUAT") {
        graphDimension = 6;
        edgesFromGraph.push_back(EdgeInformation());
        EdgeInformation& currentEdge = edgesFromGraph.back();
        currentLine >> currentEdge.fromId >> currentEdge.toId;
        currentEdge.measurement.resize(7);
        for (size_t i = 0; i < currentEdge.measurement.size(); ++i)
          currentLine >> currentEdge.measurement[i];
        currentEdge.information.resize(21);
        for (size_t i = 0; i < currentEdge.information.size(); ++i)
          currentLine >> currentEdge.information[i];
      }
    }
    assert(graphDimension > 0);
    sort(edgesFromGraph.begin(), edgesFromGraph.end(), IncrementalEdgesCompare());
    tictoc("parsing");
    cerr << "done." << endl;

    // adding edges to the graph. Add all edges connecting a node and then call optimize
    tictoc("inc_optimize");
    int lastNode = 2;
    slamInterface.addNode("", 0, graphDimension, vector<double>());
    for (vector<EdgeInformation>::const_iterator it = edgesFromGraph.begin(); it != edgesFromGraph.end(); ++it) {
      const EdgeInformation& e = *it;
      int minNodeId = max(e.fromId, e.toId);
      if (minNodeId > lastNode) {
        //cerr << "try to solve" << endl;
        lastNode = minNodeId;
        solveAndPrint(slamInterface, verbose);
      }
      //cerr << "adding " << e.fromId << " " << e.toId << endl;
      slamInterface.addEdge("", 0, graphDimension, e.fromId, e.toId, e.measurement, e.information);
    }
    solveAndPrint(slamInterface, verbose);
    tictoc("inc_optimize");
  } else {
    // Reading the protocol via stdin
    SlamParser::ParserInterface parserInterface(&slamInterface);
    while (parserInterface.parseCommand(cin)) {}
  }

  if (outputFilename.size() > 0) {
    cerr << "Saving " << outputFilename << endl;
    optimizer.save(outputFilename.c_str());
  }

  return 0;
}
