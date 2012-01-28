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

#include <signal.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>

#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

static bool hasToStop=false;

using namespace std;
using namespace g2o;

// sort according to max id, dimension, min id
struct IncrementalEdgesCompare {
  bool operator()(SparseOptimizer::Edge* const & e1, SparseOptimizer::Edge* const & e2)
  {
    const SparseOptimizer::Vertex* to1 = static_cast<const SparseOptimizer::Vertex*>(e1->vertices()[0]);
    const SparseOptimizer::Vertex* to2 = static_cast<const SparseOptimizer::Vertex*>(e2->vertices()[0]);

    int i11 = e1->vertices()[0]->id(), i12 = e1->vertices()[1]->id();
    if (i11 > i12){
      swap(i11, i12);
    }
    int i21 = e2->vertices()[0]->id(), i22 = e2->vertices()[1]->id();
    if (i21 > i22){
      swap(i21, i22);
    }
    if (i12 < i22)
      return true;
    if (i12 > i22)
      return false;
    if (to1->dimension() != to2->dimension()) { // push the odometry to be the first
      return to1->dimension() > to2->dimension();
    }
    return (i11<i21);
  }
};

SparseOptimizer::Method str2method(const std::string& strMethod_){
  string strMethod = strToLower(strMethod_);
  if (strMethod=="gauss") {
    cerr << "# Doing Gauss" << endl;
    return SparseOptimizer::GaussNewton;
  }
  if (strMethod=="levenberg") {
    cerr << "# Doing Levenberg-Marquardt" << endl;
    return SparseOptimizer::LevenbergMarquardt;
  }
  cerr << "# Unknown optimization method: " << strMethod << ", setting  default to Levenberg"  << endl;
  return SparseOptimizer::LevenbergMarquardt;
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
  int maxIterations;
  bool verbose;
  string inputFilename;
  string gnudump;
  string outputfilename;
  string strMethod;
  string strSolver;
  string loadLookup;
  bool initialGuess;
  bool marginalize;
  bool listTypes;
  bool listSolvers;
  bool incremental;
  bool guiOut;
  bool robustKernel;
  double huberWidth;
  double lambdaInit;
  int updateGraphEachN = 10;
  string statsFile;
  string dummy;
  // command line parsing
  CommandArgs arg;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("guess", initialGuess, false, "initial guess based on spanning tree");
  arg.param("inc", incremental, false, "run incremetally");
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes, (default: 10)");
  arg.param("guiout", guiOut, false, "gui output while running incrementally");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("marginalize", marginalize, false, "on or off");
  arg.param("method", strMethod, "Gauss", "Gauss or Levenberg");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("huberWidth", huberWidth, -1., "width for the robust Huber Kernel (only if robustKernel)");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "var", "specify which solver to use underneat\n\t {var, fix3_2, fix6_3, fix_7_3}");
  arg.param("solverlib", dummy, "", "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "", "specify a types library which will be loaded");
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("listSolvers", listSolvers, false, "list the available solvers");
  arg.param("renameTypes", loadLookup, "", "create a lookup for loading types into other types,\n\t TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  //OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  //DlWrapper dlSolverWrapper;
  //loadStandardSolver(dlSolverWrapper, argc, argv);
  //if (listSolvers)
    //solverFactory->listSolvers(cerr);

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(cout, true);
  }

  SparseOptimizer optimizer;
  //optimizer.setVerbose(verbose);
  //optimizer.setForceStopFlag(&hasToStop);

  // Loading the input data
  if (loadLookup.size() > 0) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.size() == 0) {
    cerr << "No input data specified" << endl;
    return 0;
  } else if (inputFilename == "-") {
    cerr << "Read input from stdin" << endl;
    if (!optimizer.load(cin)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  } else {
    cerr << "Read input from " << inputFilename << endl;
    ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      cerr << "Failed to open file" << endl;
      return 1;
    }
    if (!optimizer.load(ifs)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  }
  cerr << "Loaded " << optimizer.vertices().size() << " vertices" << endl;
  cerr << "Loaded " << optimizer.edges().size() << " edges" << endl;

  if (optimizer.vertices().size() == 0) {
    cerr << "Graph contains no vertices" << endl;
    return 1;
  }

  if (1) {
    int incIterations = maxIterations;
    int maxDim = 0;

    cerr << "# incremental setttings" << endl;
    cerr << "#\t solve every " << updateGraphEachN << endl;
    cerr << "#\t iterations  " << incIterations << endl;

    SparseOptimizer::VertexIDMap vertices = optimizer.vertices();
    for (SparseOptimizer::VertexIDMap::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
      const SparseOptimizer::Vertex* v = static_cast<const SparseOptimizer::Vertex*>(it->second);
      maxDim = (max)(maxDim, v->dimension());
    }

    vector<SparseOptimizer::Edge*> edges;
    for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
      edges.push_back(e);
    }
    optimizer.edges().clear();
    optimizer.vertices().clear();
    optimizer.setVerbose(false);

    // sort the edges in a way that inserting them makes sense
    sort(edges.begin(), edges.end(), IncrementalEdgesCompare());

    int vertexCount=0;
    int lastOptimizedVertexCount = 0;
    bool addNextEdge=true;
    bool freshlyOptimized=false;
    HyperGraph::VertexSet verticesAdded;
    int maxInGraph = -1;
    for (vector<SparseOptimizer::Edge*>::iterator it = edges.begin(); it != edges.end(); ++it) {
      SparseOptimizer::Edge* e = *it;
      bool optimize=false;

      if (addNextEdge && !optimizer.vertices().empty()){
        int idMax = (max)(e->vertices()[0]->id(), e->vertices()[1]->id());
        if (maxInGraph < idMax && ! freshlyOptimized){
          addNextEdge=false;
          optimize=true;
        } else {
          addNextEdge=true;
          optimize=false;
        }
      }

      int doInit = 0;
      SparseOptimizer::Vertex* v1 = optimizer.vertex(e->vertices()[0]->id());
      SparseOptimizer::Vertex* v2 = optimizer.vertex(e->vertices()[1]->id());
      if (! v1 && addNextEdge) {
        //cerr << " adding vertex " << it->id1 << endl;
        SparseOptimizer::Vertex* v = dynamic_cast<SparseOptimizer::Vertex*>(e->vertices()[0]);
        bool v1Added = optimizer.addVertex(v);
        maxInGraph = (max)(maxInGraph, v->id());
  //cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v1Added);
        if (! v1Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
        doInit = 1;
        if (v->dimension() == maxDim)
          vertexCount++;

        if (v->dimension() == 3) {
          cout << "ADD VERTEX_XYT " << v->id() << ";" << endl;
        }
        else if (v->dimension() == 6) {
          cout << "ADD VERTEX_XYZRPY " << v->id() << ";" << endl;
        }

      }

      if (! v2 && addNextEdge) {
        SparseOptimizer::Vertex* v = dynamic_cast<SparseOptimizer::Vertex*>(e->vertices()[1]);
        //cerr << " adding vertex " << v->id() << endl;
        bool v2Added = optimizer.addVertex(v);
        maxInGraph = (max)(maxInGraph, v->id());
  //cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v2Added);
        if (! v2Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
        doInit = 2;
        if (v->dimension() == maxDim)
          vertexCount++;
        
        if (v->dimension() == 3) {
          cout << "ADD VERTEX_XYT " << v->id() << ";" << endl;
        }
        else if (v->dimension() == 6) {
          cout << "ADD VERTEX_XYZRPY " << v->id() << ";" << endl;
        }
      }

      if (addNextEdge){

        static int edgeCnt = 0;
       
        if (e->dimension() == 3) {
          double* information = e->informationData();
          double meas[3];
          e->getMeasurementData(meas);
          //ADD EDGE_XYT 1 1 2 .1 .2 .3 1 0 0 1 0 1;
          cout << "ADD EDGE_XYT " << edgeCnt++ << " " << e->vertices()[0]->id() << " " << e->vertices()[1]->id() << " "
            << meas[0] << " " << meas[1] << " " << meas[2];
          for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j)
              cout << " " << information[i*3 + j];
          cout << ";" << endl;
        }
        else if (e->dimension() == 6) {
          // TODO convert to EULER angles
          cerr << "NOT IMPLEMENTED YET" << endl;
        }
        static bool firstEdge = true;
        if (firstEdge) {
          firstEdge = false;
          cout << "FIX 0;" << endl;
        }

        //cerr << " adding edge " << e->vertices()[0]->id() <<  " " << e->vertices()[1]->id() << endl;
        if (! optimizer.addEdge(e)) {
          cerr << "Unable to add edge " << e->vertices()[0]->id() << " -> " << e->vertices()[1]->id() << endl;
        } 
      }

      freshlyOptimized=false;
      if (optimize){
        //cerr << "Optimize" << endl;
        if (vertexCount - lastOptimizedVertexCount >= updateGraphEachN) {
          cout << "SOLVE_STATE;" << endl;
          cout << "QUERY_STATE;" << endl;
          lastOptimizedVertexCount = vertexCount;
        }

        addNextEdge=true;
        freshlyOptimized=true;
        it--;
      }

    } // for all edges

  }

  return 0;
}
