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

#include <cassert>
#include <csignal>
#include <iostream>

#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/tictoc.h"
#include "graph_optimizer_sparse_incremental.h"
#include "slam_parser/interface/parser_interface.h"

static bool hasToStop = false;

/**
 * \brief Store the information parsed from a g2o file
 */
struct EdgeInformation {
  int fromId;
  int toId;
  std::vector<double> measurement;
  std::vector<double> information;
};

/**
 * \brief Sort Edges for inserting them sequentially
 */
struct IncrementalEdgesCompare {
  bool operator()(const EdgeInformation& e1, const EdgeInformation& e2) {
    int i11 = e1.fromId;
    int i12 = e1.toId;
    if (i11 > i12) std::swap(i11, i12);
    int i21 = e2.fromId;
    int i22 = e2.toId;
    if (i21 > i22) std::swap(i21, i22);
    if (i12 < i22) return true;
    if (i12 > i22) return false;
    return i11 < i21;
  }
};

inline void solveAndPrint(g2o::G2oSlamInterface& slamInterface, bool verbose) {
  g2o::G2oSlamInterface::SolveResult solverState = slamInterface.solve();
  if (!verbose) {
    switch (solverState) {
      case g2o::G2oSlamInterface::kSolved:
        std::cout << ".";  //<< flush;
        break;
      case g2o::G2oSlamInterface::kSolvedBatch:
        std::cout << "b " << slamInterface.optimizer()->vertices().size()
                  << std::endl;
        break;
      default:
        break;
    }
  }
}

void sigquit_handler(int sig) {
  if (sig == SIGINT) {
    hasToStop = true;
    static int cnt = 0;
    if (cnt++ == 2) {
      std::cerr << __PRETTY_FUNCTION__ << " forcing exit" << std::endl;
      exit(1);
    }
  }
}

int main(int argc, char** argv) {
  std::string inputFilename;
  std::string outputFilename;
  int updateEachN;
  int batchEachN;
  bool verbose;
  bool vis;
  // command line parsing
  g2o::CommandArgs arg;
  arg.param("batch", batchEachN, 100,
            "solve by a batch Cholesky after inserting N nodes");
  arg.param("update", updateEachN, 10,
            "update the graph after inserting N nodes");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("g", vis, false, "gnuplot visualization");
  arg.param("o", outputFilename, "", "output the final graph");
  arg.param("i", inputFilename, "",
            "input file (default g2o format), if not given read via stdin");

  arg.parseArgs(argc, argv);

  g2o::SparseOptimizerIncremental optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);
  optimizer.vizWithGnuplot = vis;

  g2o::G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);
  slamInterface.setBatchSolveEachN(batchEachN);

  std::cerr << "Updating every " << updateEachN << std::endl;
  std::cerr << "Batch step every " << batchEachN << std::endl;

  if (!inputFilename.empty()) {  // operating on a file
    std::vector<EdgeInformation> edgesFromGraph;

    // HACK force tictoc statistics
#if defined _MSC_VER || defined __MINGW32__
    _putenv_s("G2O_ENABLE_TICTOC", "1");
#else
    setenv("G2O_ENABLE_TICTOC", "1", 1);
#endif

    // parse the edge from the file
    int graphDimension = 0;
    std::cerr << "Parsing " << inputFilename << " ... ";
    g2o::tictoc("parsing");
    std::ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      std::cerr << "Failure to open " << inputFilename << std::endl;
      return 1;
    }
    std::stringstream currentLine;
    while (g2o::readLine(ifs, currentLine)) {
      std::string token;
      currentLine >> token;
      if (token == "EDGE_SE2") {
        graphDimension = 3;
        edgesFromGraph.emplace_back();
        EdgeInformation& currentEdge = edgesFromGraph.back();
        currentLine >> currentEdge.fromId >> currentEdge.toId;
        currentEdge.measurement.resize(3);
        currentLine >> currentEdge.measurement[0] >>
            currentEdge.measurement[1] >> currentEdge.measurement[2];
        currentEdge.information.resize(6);
        for (int i = 0; i < 6; ++i) currentLine >> currentEdge.information[i];
      } else if (token == "EDGE_SE3:QUAT") {
        graphDimension = 6;
        edgesFromGraph.emplace_back();
        EdgeInformation& currentEdge = edgesFromGraph.back();
        currentLine >> currentEdge.fromId >> currentEdge.toId;
        currentEdge.measurement.resize(7);
        for (double& i : currentEdge.measurement) currentLine >> i;
        currentEdge.information.resize(21);
        for (double& i : currentEdge.information) currentLine >> i;
      }
    }
    assert(graphDimension > 0);
    std::sort(edgesFromGraph.begin(), edgesFromGraph.end(),
              IncrementalEdgesCompare());
    g2o::tictoc("parsing");
    std::cerr << "done." << std::endl;

    // adding edges to the graph. Add all edges connecting a node and then call
    // optimize
    g2o::tictoc("inc_optimize");
    int lastNode = 2;
    slamInterface.addNode("", 0, graphDimension, std::vector<double>());
    for (const auto& e : edgesFromGraph) {
      int minNodeId = std::max(e.fromId, e.toId);
      if (minNodeId > lastNode) {
        // std::cerr << "try to solve" << std::endl;
        lastNode = minNodeId;
        solveAndPrint(slamInterface, verbose);
      }
      // std::cerr << "adding " << e.fromId << " " << e.toId << std::endl;
      slamInterface.addEdge("", 0, graphDimension, e.fromId, e.toId,
                            e.measurement, e.information);
    }
    solveAndPrint(slamInterface, verbose);
    g2o::tictoc("inc_optimize");
  } else {
    // Reading the protocol via stdin
    slam_parser::ParserInterface parserInterface(&slamInterface);
    while (parserInterface.parseCommand(std::cin)) {
    }
  }

  if (!outputFilename.empty()) {
    std::cerr << "Saving " << outputFilename << std::endl;
    optimizer.save(outputFilename.c_str());
  }

  return 0;
}
