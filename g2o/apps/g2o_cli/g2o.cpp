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

#include <algorithm>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "dl_wrapper.h"
#include "g2o/config.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_common.h"
#include "output_helper.h"

static bool hasToStop = false;

using g2o::HyperGraph;
using g2o::OptimizableGraph;
using g2o::OptimizationAlgorithmFactory;
using g2o::SparseOptimizer;

using std::cerr;
using std::cout;
using std::endl;
using std::string;

// sort according to max id, dimension
struct IncrementalEdgesCompare {
  bool operator()(const std::shared_ptr<SparseOptimizer::Edge>& e1,
                  const std::shared_ptr<SparseOptimizer::Edge>& e2) {
    auto to1 = std::static_pointer_cast<const SparseOptimizer::Vertex>(
        e1->vertices()[1]);
    auto to2 = std::static_pointer_cast<const SparseOptimizer::Vertex>(
        e2->vertices()[1]);

    int i11 = e1->vertices()[0]->id();
    int i12 = e1->vertices()[1]->id();
    if (i11 > i12) {
      std::swap(i11, i12);
    }
    int i21 = e2->vertices()[0]->id();
    int i22 = e2->vertices()[1]->id();
    if (i21 > i22) {
      std::swap(i21, i22);
    }
    if (i12 < i22) return true;
    if (i12 > i22) return false;
    // push the odometry to be the first
    return to1->dimension() > to2->dimension();
  }
};

void sigquit_handler(int sig) {
  if (sig == SIGINT) {
    hasToStop = true;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

int main(int argc, char** argv) {
  OptimizableGraph::initMultiThreading();
  int maxIterations;
  bool verbose;
  string inputFilename;
  string gnudump;
  string outputfilename;
  string solverProperties;
  string strSolver;
  string loadLookup;
  bool initialGuess;
  bool initialGuessOdometry;
  bool marginalize;
  bool listTypes;
  bool listSolvers;
  bool listRobustKernels;
  bool incremental;
  bool guiOut;
  int gaugeId;
  string robustKernel;
  bool computeMarginals;
  bool printSolverProperties;
  double huberWidth;
  double gain;
  int maxIterationsWithGain;
  // double lambdaInit;
  int updateGraphEachN = 10;
  string statsFile;
  string summaryFile;
  bool nonSequential;
  // command line parsing
  std::vector<int> gaugeList;
  g2o::CommandArgs arg;
  arg.param("i", maxIterations, 5,
            "perform n iterations, if negative consider the gain");
  arg.param("gain", gain, 1e-6,
            "the gain used to stop optimization (default = 1e-6)");
  arg.param("ig", maxIterationsWithGain, std::numeric_limits<int>::max(),
            "Maximum number of iterations with gain enabled (default: inf)");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("guess", initialGuess, false,
            "initial guess based on spanning tree");
  arg.param("guessOdometry", initialGuessOdometry, false,
            "initial guess based on odometry");
  arg.param("inc", incremental, false, "run incremetally");
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes");
  arg.param("guiout", guiOut, false, "gui output while running incrementally");
  arg.param("marginalize", marginalize, false, "on or off");
  arg.param("printSolverProperties", printSolverProperties, false,
            "print the properties of the solver");
  arg.param("solverProperties", solverProperties, "",
            "set the internal properties of a solver,\n\te.g., "
            "initialLambda=0.0001,maxTrialsAfterFailure=2");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("robustKernel", robustKernel, "", "use this robust error function");
  arg.param("robustKernelWidth", huberWidth, -1.,
            "width for the robust Kernel (only if robustKernel)");
  arg.param("computeMarginals", computeMarginals, false,
            "computes the marginal covariances of something. FOR TESTING ONLY");
  arg.param("gaugeId", gaugeId, -1, "force the gauge");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "gn_var",
            "specify which solver to use underneat\n\t {gn_var, lm_fix3_2, "
            "gn_fix6_3, lm_fix7_3}");
#ifndef G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
  string dummy;
  arg.param("solverlib", dummy, "",
            "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "",
            "specify a types library which will be loaded");
#endif
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("listRobustKernels", listRobustKernels, false,
            "list the registered robust kernels");
  arg.param("listSolvers", listSolvers, false, "list the available solvers");
  arg.param("renameTypes", loadLookup, "",
            "create a lookup for loading types into other types,\n\t "
            "TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., "
            "VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.param("gaugeList", gaugeList, std::vector<int>(),
            "set the list of gauges separated by commas without spaces \n  "
            "e.g: 1,2,3,4,5 ");
  arg.param("summary", summaryFile, "",
            "append a summary of this optimization run to the summary file "
            "passed as argument");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed", true);
  arg.param("nonSequential", nonSequential, false,
            "apply the robust kernel only on loop closures and not odometries");

  arg.parseArgs(argc, argv);

  if (verbose) {
    cout << "# Used Compiler: " << G2O_CXX_COMPILER << endl;
  }

#ifndef G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
  // registering all the types from the libraries
  g2o::DlWrapper dlTypesWrapper;
  g2o::loadStandardTypes(dlTypesWrapper, argc, argv);
  // register all the solvers
  g2o::DlWrapper dlSolverWrapper;
  g2o::loadStandardSolver(dlSolverWrapper, argc, argv);
#else
  if (verbose) cout << "# linked version of g2o" << endl;
#endif

  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();
  if (listSolvers) {
    solverFactory->listSolvers(cout);
  }

  if (listTypes) {
    g2o::Factory::instance()->printRegisteredTypes(cout, true);
  }

  if (listRobustKernels) {
    std::vector<std::string> kernels;
    g2o::RobustKernelFactory::instance()->fillKnownKernels(kernels);
    cout << "Robust Kernels:" << endl;
    for (auto& kernel : kernels) {
      cout << kernel << endl;
    }
  }

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

  if (maxIterations < 0) {
    cerr << "# setup termination criterion based on the gain of the iteration"
         << endl;
    maxIterations = maxIterationsWithGain;
    auto terminateAction =
        std::make_shared<g2o::SparseOptimizerTerminateAction>();
    terminateAction->setGainThreshold(gain);
    terminateAction->setMaxIterations(maxIterationsWithGain);
    optimizer.addPostIterationAction(terminateAction);
  }

  // allocating the desired solver + testing whether the solver is okay
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(solverFactory->construct(strSolver, solverProperty));
  if (!optimizer.solver()) {
    cerr << "Error allocating solver. Allocating \"" << strSolver
         << "\" failed!" << endl;
    return 0;
  }

  if (!solverProperties.empty()) {
    bool updateStatus =
        optimizer.solver()->updatePropertiesFromString(solverProperties);
    if (!updateStatus) {
      cerr << "Failure while updating the solver properties from the given "
              "string"
           << endl;
    }
  }
  if (!solverProperties.empty() || printSolverProperties) {
    optimizer.solver()->printProperties(cerr);
  }

  // Loading the input data
  if (!loadLookup.empty()) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.empty()) {
    cerr << "No input data specified" << endl;
    return 0;
  }
  if (inputFilename == "-") {
    cerr << "Read input from stdin" << endl;
    if (!optimizer.load(std::cin)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  } else {
    cerr << "Read input from " << inputFilename << endl;
    std::ifstream ifs(inputFilename.c_str());
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

  if (optimizer.vertices().empty()) {
    cerr << "Graph contains no vertices" << endl;
    return 1;
  }

  std::set<int> vertexDimensions = optimizer.dimensions();
  if (!optimizer.isSolverSuitable(solverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given graph"
         << endl;
    return 3;
  }
  assert(optimizer.solver());
  // optimizer.setMethod(str2method(strMethod));
  // optimizer.setUserLambdaInit(lambdaInit);

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = optimizer.gaugeFreedom();
  std::shared_ptr<OptimizableGraph::Vertex> gauge;
  if (!gaugeList.empty()) {
    cerr << "Fixing gauges: ";
    for (size_t i = 0; i < gaugeList.size(); i++) {
      int id = gaugeList[i];
      auto v = optimizer.vertex(id);
      if (!v) {
        cerr << "fatal, not found the vertex of id " << id
             << " in the gaugeList. Aborting";
        return -1;
      }
      if (i == 0) gauge = v;
      cerr << v->id() << " ";
      v->setFixed(true);
    }
    cerr << endl;
    gaugeFreedom = false;
  } else {
    gauge = optimizer.findGauge();
  }
  if (gaugeFreedom) {
    if (!gauge) {
      cerr << "# cannot find a vertex to fix in this thing" << endl;
      return 2;
    }
    cerr << "# graph is fixed by node " << gauge->id() << endl;
    gauge->setFixed(true);

  } else {
    cerr << "# graph is fixed by priors or already fixed vertex" << endl;
  }

  // if schur, we wanna marginalize the landmarks...
  if (marginalize || solverProperty.requiresMarginalize) {
    int maxDim = *vertexDimensions.rbegin();
    int minDim = *vertexDimensions.begin();
    if (maxDim != minDim) {
      cerr << "# Preparing Marginalization of the Landmarks ... ";
      for (auto& it : optimizer.vertices()) {
        auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(it.second);
        if (v->dimension() != maxDim) {
          v->setMarginalized(true);
        }
      }
      cerr << "done." << endl;
    }
  }

  if (!robustKernel.empty()) {
    g2o::AbstractRobustKernelCreator::Ptr creator =
        g2o::RobustKernelFactory::instance()->creator(robustKernel);
    cerr << "# Preparing robust error function ... ";
    g2o::RobustKernelPtr robustKernel = creator->construct();
    if (huberWidth > 0) robustKernel->setDelta(huberWidth);
    if (creator) {
      if (nonSequential) {
        for (const auto& it : optimizer.edges()) {
          auto e = std::dynamic_pointer_cast<SparseOptimizer::Edge>(it);
          if (e->vertices().size() >= 2 &&
              std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
            e->setRobustKernel(robustKernel);
          }
        }
      } else {
        for (const auto& it : optimizer.edges()) {
          auto e = std::dynamic_pointer_cast<SparseOptimizer::Edge>(it);
          e->setRobustKernel(robustKernel);
        }
      }
      cerr << "done." << endl;
    } else {
      cerr << "Unknown Robust Kernel: " << robustKernel << endl;
    }
  }

  // sanity check
  auto optimizerWrapper =
      std::shared_ptr<HyperGraph>(&optimizer, [](HyperGraph*) {});
  g2o::HyperDijkstra d(optimizerWrapper);
  g2o::UniformCostFunction f;
  d.shortestPaths(gauge, f);
  // cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size() != optimizer.vertices().size()) {
    cerr << CL_RED("Warning: d.visited().size() != optimizer.vertices().size()")
         << endl;
    cerr << "visited: " << d.visited().size() << endl;
    cerr << "vertices: " << optimizer.vertices().size() << endl;
  }

  if (incremental) {
    cerr << CL_RED(
                "# Note: this variant performs batch steps in each time step")
         << endl;
    cerr << CL_RED(
                "#       For a variant which updates the Cholesky factor use "
                "the binary g2o_incremental")
         << endl;
    int incIterations = maxIterations;
    if (!arg.parsedParam("i")) {
      cerr << "# Setting default number of iterations" << endl;
      incIterations = 1;
    }
    int updateDisplayEveryN = updateGraphEachN;
    int maxDim = 0;

    cerr << "# incremental settings" << endl;
    cerr << "#\t solve every " << updateGraphEachN << endl;
    cerr << "#\t iterations  " << incIterations << endl;

    SparseOptimizer::VertexIDMap vertices = optimizer.vertices();
    for (auto& vertice : vertices) {
      auto v =
          std::static_pointer_cast<SparseOptimizer::Vertex>(vertice.second);
      maxDim = std::max(maxDim, v->dimension());
    }

    SparseOptimizer::EdgeContainer edges;
    for (const auto& it : optimizer.edges()) {
      edges.emplace_back(std::dynamic_pointer_cast<SparseOptimizer::Edge>(it));
    }
    optimizer.edges().clear();
    optimizer.vertices().clear();
    optimizer.setVerbose(false);

    // sort the edges in a way that inserting them makes sense
    sort(edges.begin(), edges.end(), IncrementalEdgesCompare());

    double cumTime = 0.;
    int vertexCount = 0;
    int lastOptimizedVertexCount = 0;
    int lastVisUpdateVertexCount = 0;
    bool freshlyOptimized = false;
    bool firstRound = true;
    HyperGraph::VertexSet verticesAdded;
    HyperGraph::EdgeSet edgesAdded;
    for (auto& e : edges) {
      int doInit = 0;
      auto v1 = optimizer.vertex(e->vertices()[0]->id());
      auto v2 = optimizer.vertex(e->vertices()[1]->id());

      if (!v1) {
        auto v = v1 = std::dynamic_pointer_cast<SparseOptimizer::Vertex>(
            e->vertices()[0]);
        bool v1Added = optimizer.addVertex(v);
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v1Added);
        if (!v1Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
        doInit = 1;
        if (v->dimension() == maxDim) vertexCount++;
      }

      if (!v2) {
        auto v = v2 = std::dynamic_pointer_cast<SparseOptimizer::Vertex>(
            e->vertices()[1]);
        bool v2Added = optimizer.addVertex(v);
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v2Added);
        if (!v2Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
        doInit = 2;
        if (v->dimension() == maxDim) vertexCount++;
      }

      // adding the edge and initialization of the vertices
      {
        // cerr << " adding edge " << e->vertices()[0]->id() <<  " " <<
        // e->vertices()[1]->id() << endl;
        if (!optimizer.addEdge(e)) {
          cerr << "Unable to add edge " << e->vertices()[0]->id() << " -> "
               << e->vertices()[1]->id() << endl;
        } else {
          edgesAdded.insert(e);
        }

        if (doInit) {
          auto from = std::static_pointer_cast<OptimizableGraph::Vertex>(
              e->vertices()[0]);
          auto to = std::static_pointer_cast<OptimizableGraph::Vertex>(
              e->vertices()[1]);
          switch (doInit) {
            case 1:  // initialize v1 from v2
            {
              HyperGraph::VertexSet toSet;
              toSet.insert(to);
              if (e->initialEstimatePossible(toSet, from.get()) > 0.) {
                // cerr << "init: "
                //<< to->id() << "(" << to->dimension() << ") -> "
                //<< from->id() << "(" << from->dimension() << ") " << endl;
                e->initialEstimate(toSet, from.get());
              } else {
                assert(0 && "Added uninitialized variable to the graph");
              }
              break;
            }
            case 2: {
              HyperGraph::VertexSet fromSet;
              fromSet.insert(from);
              if (e->initialEstimatePossible(fromSet, to.get()) > 0.) {
                // cerr << "init: "
                //<< from->id() << "(" << from->dimension() << ") -> "
                //<< to->id() << "(" << to->dimension() << ") " << endl;
                e->initialEstimate(fromSet, to.get());
              } else {
                assert(0 && "Added uninitialized variable to the graph");
              }
              break;
            }
            default:
              cerr << "doInit wrong value\n";
          }
        }
      }

      freshlyOptimized = false;
      {
        // cerr << "Optimize" << endl;
        if (vertexCount - lastOptimizedVertexCount >= updateGraphEachN) {
          if (firstRound) {
            if (!optimizer.initializeOptimization()) {
              cerr << "initialization failed" << endl;
              return 0;
            }
          } else {
            if (!optimizer.updateInitialization(verticesAdded, edgesAdded)) {
              cerr << "updating initialization failed" << endl;
              return 0;
            }
          }
          verticesAdded.clear();
          edgesAdded.clear();
          double ts = g2o::get_monotonic_time();
          int currentIt = optimizer.optimize(incIterations, !firstRound);
          double dts = g2o::get_monotonic_time() - ts;
          cumTime += dts;
          firstRound = false;
          // optimizer->setOptimizationTime(cumTime);
          if (verbose) {
            double chi2 = optimizer.chi2();
            cerr << "nodes= " << optimizer.vertices().size()
                 << "\t edges= " << optimizer.edges().size()
                 << "\t chi2= " << chi2 << "\t time= " << dts
                 << "\t iterations= " << currentIt << "\t cumTime= " << cumTime
                 << endl;
          }
          lastOptimizedVertexCount = vertexCount;
          freshlyOptimized = true;

          if (guiOut) {
            if (vertexCount - lastVisUpdateVertexCount >= updateDisplayEveryN) {
              dumpEdges(cout, optimizer);
              lastVisUpdateVertexCount = vertexCount;
            }
          }
        }

        if (!verbose) cerr << ".";
      }

    }  // for all edges

    if (!freshlyOptimized) {
      double ts = g2o::get_monotonic_time();
      int currentIt = optimizer.optimize(incIterations, !firstRound);
      double dts = g2o::get_monotonic_time() - ts;
      cumTime += dts;
      // optimizer->setOptimizationTime(cumTime);
      if (verbose) {
        double chi2 = optimizer.chi2();
        cerr << "nodes= " << optimizer.vertices().size()
             << "\t edges= " << optimizer.edges().size() << "\t chi2= " << chi2
             << "\t time= " << dts << "\t iterations= " << currentIt
             << "\t cumTime= " << cumTime << endl;
      }
    }

  } else {
    // BATCH optimization

    if (!statsFile.empty()) {
      // allocate buffer for statistics;
      optimizer.setComputeBatchStatistics(true);
    }
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    double loadChi = optimizer.chi2();
    cerr << "Initial chi2 = " << FIXED(loadChi) << endl;

    if (initialGuess) {
      optimizer.computeInitialGuess();
    } else if (initialGuessOdometry) {
      g2o::EstimatePropagatorCostOdometry costFunction(&optimizer);
      optimizer.computeInitialGuess(costFunction);
    }
    double initChi = optimizer.chi2();

    signal(SIGINT, sigquit_handler);
    int result = optimizer.optimize(maxIterations);
    if (maxIterations > 0 && result == g2o::OptimizationAlgorithm::kFail) {
      cerr << "Cholesky failed, result might be invalid" << endl;
    } else if (computeMarginals) {
      std::vector<std::pair<int, int> > blockIndices;
      for (const auto& v : optimizer.activeVertices()) {
        if (v->hessianIndex() >= 0) {
          blockIndices.emplace_back(v->hessianIndex(), v->hessianIndex());
        }
        if (v->hessianIndex() > 0) {
          blockIndices.emplace_back(v->hessianIndex() - 1, v->hessianIndex());
        }
      }
      g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;
      if (optimizer.computeMarginals(spinv, blockIndices)) {
        for (const auto& v : optimizer.activeVertices()) {
          cerr << "Vertex id:" << v->id() << endl;
          if (v->hessianIndex() >= 0) {
            cerr << "inv block :" << v->hessianIndex() << ", "
                 << v->hessianIndex() << endl;
            cerr << *(spinv.block(v->hessianIndex(), v->hessianIndex()));
            cerr << endl;
          }
          if (v->hessianIndex() > 0) {
            cerr << "inv block :" << v->hessianIndex() - 1 << ", "
                 << v->hessianIndex() << endl;
            cerr << *(spinv.block(v->hessianIndex() - 1, v->hessianIndex()));
            cerr << endl;
          }
        }
      }
    }

    optimizer.computeActiveErrors();
    double finalChi = optimizer.chi2();

    if (!summaryFile.empty()) {
      g2o::PropertyMap summary;
      summary.makeProperty<g2o::StringProperty>("filename", inputFilename);
      summary.makeProperty<g2o::IntProperty>("n_vertices",
                                             optimizer.vertices().size());
      summary.makeProperty<g2o::IntProperty>("n_edges",
                                             optimizer.edges().size());

      int nLandmarks = 0;
      int nPoses = 0;
      int maxDim = *vertexDimensions.rbegin();
      for (auto& it : optimizer.vertices()) {
        auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(it.second);
        if (v->dimension() != maxDim) {
          nLandmarks++;
        } else
          nPoses++;
      }
      std::set<string> edgeTypes;
      for (const auto& it : optimizer.edges()) {
        edgeTypes.insert(g2o::Factory::instance()->tag(it.get()));
      }
      std::stringstream edgeTypesString;
      for (const auto& edgeType : edgeTypes) {
        edgeTypesString << edgeType << " ";
      }

      summary.makeProperty<g2o::IntProperty>("n_poses", nPoses);
      summary.makeProperty<g2o::IntProperty>("n_landmarks", nLandmarks);
      summary.makeProperty<g2o::StringProperty>("edge_types",
                                                edgeTypesString.str());
      summary.makeProperty<g2o::DoubleProperty>("load_chi", loadChi);
      summary.makeProperty<g2o::StringProperty>("solver", strSolver);
      summary.makeProperty<g2o::BoolProperty>("robustKernel",
                                              !robustKernel.empty());
      summary.makeProperty<g2o::DoubleProperty>("init_chi", initChi);
      summary.makeProperty<g2o::DoubleProperty>("final_chi", finalChi);
      summary.makeProperty<g2o::IntProperty>("maxIterations", maxIterations);
      summary.makeProperty<g2o::IntProperty>("realIterations", result);
      std::ofstream os;
      os.open(summaryFile.c_str(), std::ios::app);
      summary.writeToCSV(os);
    }

    if (!statsFile.empty()) {
      cerr << "writing stats to file \"" << statsFile << "\" ... ";
      std::ofstream os(statsFile.c_str());
      const g2o::BatchStatisticsContainer& bsc = optimizer.batchStatistics();

      for (int i = 0; i < maxIterations; i++) {
        os << bsc[i] << endl;
      }
      cerr << "done." << endl;
    }
  }

  // saving again
  if (!gnudump.empty()) {
    bool gnuPlotStatus = saveGnuplot(gnudump, optimizer);
    if (!gnuPlotStatus) {
      cerr << "Error while writing gnuplot files" << endl;
    }
  }

  if (!outputfilename.empty()) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      optimizer.save(cout);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      optimizer.save(outputfilename.c_str());
    }
    cerr << "done." << endl;
  }

  return 0;
}
