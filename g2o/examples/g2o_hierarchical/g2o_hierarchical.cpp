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
#include <csignal>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>

#include "CLI/CLI.hpp"
#include "edge_creator.h"
#include "edge_labeler.h"
#include "edge_types_cost_function.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel.h"  // IWYU pragma: keep
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/macros.h"
#include "star.h"
// #include "backbone_tree_action.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "simple_star_ops.h"

using std::cerr;
using std::string;

namespace {
bool hasToStop = false;

void sigquit_handler(int sig) {
  if (sig == SIGINT) {
    hasToStop = true;
    static int cnt = 0;
    if (cnt++ == 2) {
      G2O_WARN("forcing exit");
      exit(1);
    }
  }
}
}  // namespace

/*
// string newTypesMapping=
// "VERTEX_SE3:QUAT=VERTEX_SE3_NEW,\
// VERTEX_TRACKXYZ=VERTEX_TRACKXYZ_NEW,\
// PARAMS_SE3OFFSET=PARAMS_SE3OFFSET_NEW,\
// PARAMS_CAMERACALIB=PARAMS_CAMERACALIB_NEW,\
// EDGE_SE3:QUAT=EDGE_SE3_NEW,\
// EDGE_SE3_TRACKXYZ=EDGE_SE3_TRACKXYZ_NEW,\
// EDGE_SE3_OFFSET=EDGE_SE3_OFFSET_NEW,\
// EDGE_PROJECT_DISPARITY=EDGE_PROJECT_DISPARITY_NEW,\
// EDGE_PROJECT_DEPTH=EDGE_PROJECT_DEPTH_NEW,\
// CACHE_CAMERA=CACHE_CAMERA_NEW,\
// CACHE_SE3_OFFSET=CACHE_SE3_OFFSET_NEW";
*/

namespace g2o {
namespace {

int run_hierarchical(int argc, char** argv) {
  int starIterations = 30;
  int highIterations = 100;
  int lowIterations = 100;
  bool verbose;
  // bool useNewTypes;
  string inputFilename;
  string gnudump;
  string outputfilename;
  // string strMethod;
  string strSolver = "lm_var_cholmod";
  string strHSolver = "gn_var_cholmod";
  string loadLookup;
  bool initialGuess;
  bool listTypes;
  bool listSolvers;
  bool listRobustKernels;
  bool guiOut;
  bool computeMarginals;
  double huberWidth = -1;
  bool debug;
  double uThreshold = -1.;
  string robustKernel;
  // double lambdaInit;
  int hierarchicalDiameter = -1;
  int updateGraphEachN = 10;
  string summaryFile;
  string dummy;
  // command line parsing
  CLI::App app{"g2o's CLI"};
  argv = app.ensure_utf8(argv);
  app.add_option("--si", starIterations,
                 "perform n iterations to build the stars");
  app.add_option("--hi", highIterations,
                 "perform n iterations to construct the hierarchy");
  app.add_option("--li", lowIterations,
                 "perform n iterations on the low level");
  app.add_flag("-v", verbose, "verbose output of the optimization process");
  app.add_option("--u_threshold", uThreshold,
                 "rejection threshold for underdetermined vertices");
  app.add_option("--hierarchical_diameter", hierarchicalDiameter,
                 "selects the diameter of the stars in the hierarchical graph");
  app.add_flag("--guess", initialGuess, "initial guess based on spanning tree");
  // arg.param("useNewTypes", useNewTypes, false, "if true remaps the slam3d old
  // types into the new ones");
  app.add_flag("--debug", debug, "print shit load of things for debugging");
  app.add_option("--update", updateGraphEachN,
                 "updates after x odometry nodes, (default: 10)");
  app.add_flag("--guiout", guiOut, "gui output while running incrementally");
  app.add_option("--gnudump", gnudump, "dump to gnuplot data file");
  app.add_option("--robust_kernel", robustKernel,
                 "use this robust error function");
  app.add_option("--robust_kernel_width", huberWidth,
                 "width for the robust Kernel (only if robustKernel)");
  app.add_option(
      "--compute_marginals", computeMarginals,
      "computes the marginal covariances of something. FOR TESTING ONLY");
  app.add_option("-o,--output", outputfilename,
                 "output final version of the graph");
  app.add_option("--solver", strSolver,
                 "specify which solver to use underneat");
  app.add_option("--hsolver", strHSolver,
                 "specify which solver to use for the high level");
  app.add_option("--solverlib", dummy,
                 "specify a solver library which will be loaded");
  app.add_option("--typeslib", dummy,
                 "specify a types library which will be loaded");
  app.add_flag("--list_types", listTypes, "list the registered types");
  app.add_flag("--list_solvers", listSolvers, "list the available solvers");
  app.add_flag("--list_robust_kernels", listRobustKernels,
               "list the registered robust kernels");

  app.add_option("--rename_types", loadLookup,
                 "create a lookup for loading types into other types, "
                 "TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2, e.g., "
                 "VERTEX_CAM=VERTEX_SE3:EXPMAP");
  app.add_option(
      "--summary", summaryFile,
      "append a summary of this optimization run to the summary file "
      "passed as argument");
  app.add_option("graph-input", inputFilename,
                 "graph file which will be processed ('-' for stdin)")
      ->required();

  CLI11_PARSE(app, argc, argv);

  // if (useNewTypes){
  //   loadLookup=newTypesMapping+loadLookup;
  // }
  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);
  if (listSolvers) solverFactory->listSolvers(cerr);

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(std::cout, true);
  }

  if (listRobustKernels) {
    std::vector<std::string> kernels;
    RobustKernelFactory::instance()->fillKnownKernels(kernels);
    std::cout << "Robust Kernels:\n";
    for (auto& kernel : kernels) {
      std::cout << kernel << '\n';
    }
  }

  AbstractRobustKernelCreator::Ptr kernelCreator;
  if (!robustKernel.empty()) {
    kernelCreator = RobustKernelFactory::instance()->creator(robustKernel);
  }

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

  // Loading the input data
  if (!loadLookup.empty()) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.empty()) {
    cerr << "No input data specified\n";
    return 0;
  }
  if (inputFilename == "-") {
    cerr << "Read input from stdin\n";
    if (!optimizer.load(std::cin)) {
      cerr << "Error loading graph\n";
      return 2;
    }
  } else {
    cerr << "Read input from " << inputFilename << '\n';
    std::ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      cerr << "Failed to open file\n";
      return 1;
    }
    if (!optimizer.load(ifs)) {
      cerr << "Error loading graph\n";
      return 2;
    }
  }
  cerr << "Loaded " << optimizer.vertices().size() << " vertices\n";
  cerr << "Loaded " << optimizer.edges().size() << " edges\n";

  OptimizableGraph::EdgeSet originalEdges = optimizer.edges();

  EdgeCreator creator;
  creator.addAssociation("VERTEX_SE2;VERTEX_SE2;", "EDGE_SE2");
  creator.addAssociation("VERTEX_SE2;VERTEX_XY;", "EDGE_SE2_XY");
  creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_SE3:QUAT;", "EDGE_SE3:QUAT");
  creator.addAssociation("VERTEX_SE3_NEW;VERTEX_SE3_NEW;", "EDGE_SE3_NEW");

  std::shared_ptr<Parameter> p0 = optimizer.parameter(0);
  if (p0) {
    auto* originalParams = dynamic_cast<ParameterSE3Offset*>(p0.get());
    if (originalParams) {
      cerr << "ORIGINAL PARAMS\n";
      auto se3OffsetParam = std::make_shared<ParameterSE3Offset>();
      se3OffsetParam->setId(100);
      optimizer.addParameter(se3OffsetParam);
      std::vector<int> depthCamHParamsIds(1);
      depthCamHParamsIds[0] = se3OffsetParam->id();
      creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_TRACKXYZ;",
                             "EDGE_SE3_TRACKXYZ", depthCamHParamsIds);
    }
  }

  EdgeLabeler labeler(&optimizer);

  if (optimizer.vertices().empty()) {
    cerr << "Graph contains no vertices\n";
    return 1;
  }

  // allocating the desired solver + testing whether the solver is okay
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithmProperty hsolverProperty;
  std::shared_ptr<OptimizationAlgorithm> solver =
      solverFactory->construct(strSolver, solverProperty);
  std::shared_ptr<OptimizationAlgorithm> hsolver =
      solverFactory->construct(strHSolver, hsolverProperty);
  if (!solver) {
    cerr << "Error allocating solver. Allocating \"" << strSolver
         << "\" failed!\n";
    return 0;
  }
  if (!hsolver) {
    cerr << "Error allocating hsolver. Allocating \"" << strHSolver
         << "\" failed!\n";
    return 0;
  }

  const std::unordered_set<int> vertexDimensions = optimizer.dimensions();
  if (!optimizer.isSolverSuitable(solverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given "
            "graph\n";
    return 3;
  }
  if (!optimizer.isSolverSuitable(hsolverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given "
            "graph\n";
    return 3;
  }

  optimizer.setAlgorithm(solver);

  const int poseDim =
      *std::max_element(vertexDimensions.begin(), vertexDimensions.end());
  string backboneVertexType;
  string backboneEdgeType;
  switch (poseDim) {
    case 3:
      if (hierarchicalDiameter == -1) hierarchicalDiameter = 30;
      backboneEdgeType = "EDGE_SE2";
      backboneVertexType = "VERTEX_SE2";
      if (uThreshold < 0) {
        uThreshold = 1e-5;
      }
      break;
    case 6:
      if (hierarchicalDiameter == -1) hierarchicalDiameter = 4;
      backboneEdgeType = "EDGE_SE3:QUAT";
      backboneVertexType = "VERTEX_SE3:QUAT";
      // if (useNewTypes){
      //   backboneEdgeType =   "EDGE_SE3_NEW";
      //   backboneVertexType = "VERTEX_SE3_NEW";
      // }

      if (uThreshold < 0) {
        uThreshold = 1e-3;
      }
      break;
    default:
      cerr << "Fatal: unknown backbone type. The largest vertex dimension is: "
           << poseDim << ".\nExiting.\n";
      return -1;
  }

  // here we need to chop the graph into many lil pieces

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = optimizer.gaugeFreedom();
  std::shared_ptr<OptimizableGraph::Vertex> gauge = optimizer.findGauge();

  if (gaugeFreedom) {
    if (!gauge) {
      cerr << "# cannot find a vertex to fix in this thing\n";
      return 2;
    }
    cerr << "# graph is fixed by node " << gauge->id() << '\n';
    gauge->setFixed(true);

  } else {
    cerr << "# graph is fixed by priors\n";
  }

  // sanity check
  auto pointerWrapper =
      std::shared_ptr<HyperGraph>(&optimizer, [](HyperGraph*) {});
  HyperDijkstra d(pointerWrapper);
  UniformCostFunction f;
  d.shortestPaths(gauge, f);
  // cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size() != optimizer.vertices().size()) {
    cerr << "Warning: d.visited().size() != optimizer.vertices().size()\n";
    cerr << "visited: " << d.visited().size() << '\n';
    cerr << "vertices: " << optimizer.vertices().size() << '\n';
  }

  // BATCH optimization

  optimizer.initializeOptimization();

  optimizer.computeActiveErrors();
  double loadChi = optimizer.activeChi2();

  cerr << "Initial chi2 = " << FIXED(loadChi) << '\n';

  if (initialGuess) optimizer.computeInitialGuess();
  signal(SIGINT, sigquit_handler);

  optimizer.computeActiveErrors();
  double initChi = optimizer.activeChi2();

  //  if (robustKernel) {
  // cerr << "# Preparing robust error function ... ";
  for (const auto& it : optimizer.edges()) {
    auto e = std::dynamic_pointer_cast<SparseOptimizer::Edge>(it);
    if (kernelCreator) {
      e->setRobustKernel(kernelCreator->construct());
      if (huberWidth > 0) e->robustKernel()->setDelta(huberWidth);
    }
  }
  // cerr << "done." << endl;
  // }
  optimizer.computeActiveErrors();

  StarSet stars;

  computeSimpleStars(stars, &optimizer, &labeler, &creator, gauge,
                     backboneEdgeType, backboneVertexType, 0,
                     hierarchicalDiameter, 1, starIterations, uThreshold,
                     debug);

  cerr << "stars computed, stars.size()= " << stars.size() << '\n';

  cerr << "hierarchy done, determining border\n";
  EdgeStarMap hesmap;
  constructEdgeStarMap(hesmap, stars, false);
  computeBorder(stars, hesmap);

  OptimizableGraph::EdgeSet eset;
  OptimizableGraph::VertexSet vset;
  OptimizableGraph::EdgeSet heset;
  OptimizableGraph::VertexSet hvset;
  HyperGraph::VertexSet hgauge;
  for (const auto& s : stars) {
    if (hgauge.empty()) hgauge = s->gauge();

    for (const auto& git : s->gauge()) {
      hvset.insert(git);
    }

    for (const auto& e : s->starEdges()) {
      eset.insert(e);
      for (auto& i : e->vertices()) {
        vset.insert(i);
      }
    }
    for (const auto& iit : s->starFrontierEdges()) {
      heset.insert(iit);
    }
  }
  cerr << "eset.size()= " << eset.size() << '\n';
  cerr << "heset.size()= " << heset.size() << '\n';

  std::ofstream starStream("stars.g2o");
  optimizer.saveSubset(starStream, eset);
  starStream.close();

  std::ofstream hstarStream("hstars.g2o");
  optimizer.saveSubset(hstarStream, heset);
  hstarStream.close();

  cerr << "stars done!\n";

  cerr << "optimizing the high layer\n";
  for (const auto& it : hgauge) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(true);
  }
  optimizer.setAlgorithm(hsolver);
  optimizer.initializeOptimization(heset);
  optimizer.setVerbose(true);
  if (initialGuess) optimizer.computeInitialGuess();

  optimizer.computeActiveErrors();
  double hInitChi = optimizer.activeChi2();

  optimizer.optimize(highIterations);

  optimizer.computeActiveErrors();
  double hFinalChi = optimizer.activeChi2();

  cerr << "done\n";

  if (!kernelCreator) {
    cerr << "# Robust error function disabled ";
    for (const auto& it : optimizer.edges()) {
      auto* e = dynamic_cast<SparseOptimizer::Edge*>(it.get());
      e->setRobustKernel(nullptr);
    }
    cerr << "done.\n";
  } else {
    cerr << "# Preparing robust error function ay low level done";
  }

  cerr << "fixing the hstructure, and optimizing the floating nodes\n";
  for (const auto& it : hvset) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(true);
  }
  optimizer.initializeOptimization(eset);
  optimizer.computeInitialGuess();
  optimizer.optimize(1);
  cerr << "done\n";
  if (debug) {
    std::ofstream os("debug_low_level.g2o");
    optimizer.saveSubset(os, eset);
  }

  cerr << "adding the original constraints, locking hierarchical solution and "
          "optimizing the free variables\n";
  for (const auto& it : vset) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(true);
  }
  for (const auto& it : hgauge) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(true);
  }
  optimizer.setAlgorithm(solver);
  optimizer.initializeOptimization(0);
  optimizer.computeInitialGuess();
  optimizer.optimize(lowIterations);

  cerr << "relaxing the full problem\n";
  for (const auto& it : vset) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(false);
  }
  for (const auto& it : hgauge) {
    auto* g = dynamic_cast<OptimizableGraph::Vertex*>(it.get());
    g->setFixed(true);
  }
  optimizer.initializeOptimization(0);
  int result = optimizer.optimize(lowIterations);
  if (result < 0) cerr << "failure in low level optimization\n";

  optimizer.computeActiveErrors();
  double finalChi = optimizer.activeChi2();

  if (!summaryFile.empty()) {
    PropertyMap summary;
    summary.makeProperty<StringProperty>("filename", inputFilename);
    summary.makeProperty<IntProperty>("n_vertices",
                                      optimizer.vertices().size());

    int nLandmarks = 0;
    int nPoses = 0;
    const int maxDim =
        *std::max_element(vertexDimensions.begin(), vertexDimensions.end());
    for (auto& it : optimizer.vertices()) {
      auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
      if (v->dimension() != maxDim) {
        nLandmarks++;
      } else
        nPoses++;
    }

    int nEdges = 0;
    std::set<string> edgeTypes;
    for (const auto& it : optimizer.edges()) {
      auto* e = dynamic_cast<OptimizableGraph::Edge*>(it.get());
      if (e->level() == 0) {
        edgeTypes.insert(Factory::instance()->tag(e));
        nEdges++;
      }
    }
    std::stringstream edgeTypesString;
    for (const auto& edgeType : edgeTypes) {
      edgeTypesString << edgeType << " ";
    }

    summary.makeProperty<IntProperty>("n_edges", nEdges);
    summary.makeProperty<IntProperty>("n_poses", nPoses);
    summary.makeProperty<IntProperty>("n_landmarks", nLandmarks);
    summary.makeProperty<StringProperty>("edge_types", edgeTypesString.str());
    summary.makeProperty<DoubleProperty>("load_chi", loadChi);
    summary.makeProperty<DoubleProperty>("init_chi", initChi);
    summary.makeProperty<DoubleProperty>("final_chi", finalChi);
    summary.makeProperty<StringProperty>("solver", strSolver);
    summary.makeProperty<StringProperty>("robustKernel", robustKernel);

    summary.makeProperty<IntProperty>("n_stars", stars.size());
    summary.makeProperty<IntProperty>("n_star_edges", eset.size());
    summary.makeProperty<IntProperty>("n_star_h_edges", heset.size());
    summary.makeProperty<IntProperty>("n_star_h_vertices", hvset.size());
    summary.makeProperty<DoubleProperty>("h_initChi", hInitChi);
    summary.makeProperty<DoubleProperty>("h_finalChi", hFinalChi);

    std::ofstream os;
    os.open(summaryFile.c_str(), std::ios::app);
    summary.writeToCSV(os);
  }

  if (!outputfilename.empty()) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      optimizer.saveSubset(std::cout, originalEdges);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      std::ofstream os(outputfilename.c_str());
      optimizer.saveSubset(os, originalEdges);
    }
    cerr << "done.\n";
  }

  // destroy all the singletons
  // Factory::destroy();
  // OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();

  return 0;
}
}  // namespace

}  // namespace g2o

int main(int argc, char** argv) { return g2o::run_hierarchical(argc, argv); }
