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
#include <sstream>
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "edge_labeler.h"
#include "edge_creator.h"
#include "edge_types_cost_function.h"
#include "star.h"
//#include "backbone_tree_action.h"
#include "simple_star_ops.h"


#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

// #include "g2o/types/slam3d_new/parameter_camera.h"
// #include "g2o/types/slam3d_new/parameter_se3_offset.h"

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
int main(int argc, char** argv)
{

  int starIterations;
  int highIterations;
  int lowIterations;
  bool verbose;
  //bool useNewTypes;
  string inputFilename;
  string gnudump;
  string outputfilename;
  //string strMethod;
  string strSolver;
  string strHSolver;
  string loadLookup;
  bool initialGuess;
  bool listTypes;
  bool listSolvers;
  bool listRobustKernels;
  bool guiOut;
  bool computeMarginals;
  double huberWidth;
  bool debug;
  double uThreshold;
  string robustKernel;
  //double lambdaInit;
  int hierarchicalDiameter;
  int updateGraphEachN = 10;
  string summaryFile;
  string dummy;
  // command line parsing
  CommandArgs arg;
  arg.param("si", starIterations, 30, "perform n iterations to build the stars");
  arg.param("hi", highIterations, 100, "perform n iterations to construct the hierarchy");
  arg.param("li", lowIterations, 100, "perform n iterations on the low level");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("uThreshold", uThreshold, -1., "rejection threshold for underdetermined vertices");
  arg.param("hierarchicalDiameter", hierarchicalDiameter, -1 , "selects the diameter of the stars in the hierarchical graph");
  arg.param("guess", initialGuess, false, "initial guess based on spanning tree");
  //arg.param("useNewTypes", useNewTypes, false, "if true remaps the slam3d old types into the new ones");
  arg.param("debug", debug, false, "print shit load of things for debugging");
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes, (default: 10)");
  arg.param("guiout", guiOut, false, "gui output while running incrementally");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("robustKernel", robustKernel, "", "use this robust error function");
  arg.param("robustKernelWidth", huberWidth, -1., "width for the robust Kernel (only if robustKernel)");
  arg.param("computeMarginals", computeMarginals, false, "computes the marginal covariances of something. FOR TESTING ONLY");
  arg.param("huberWidth", huberWidth, -1., "width for the robust Huber Kernel (only if robustKernel)");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "lm_var_cholmod", "specify which solver to use underneat");
  arg.param("hsolver", strHSolver, "gn_var_cholmod", "specify which solver to use for the high level");
  arg.param("solverlib", dummy, "", "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "", "specify a types library which will be loaded");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("listSolvers", listSolvers, false, "list the available solvers");
  arg.param("listRobustKernels", listRobustKernels, false, "list the registered robust kernels");

  arg.param("renameTypes", loadLookup, "", "create a lookup for loading types into other types,\n\t TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.param("summary", summaryFile, "", "append a summary of this optimization run to the summary file passed as argument");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  // if (useNewTypes){
  //   loadLookup=newTypesMapping+loadLookup;
  // }
  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);
  if (listSolvers)
    solverFactory->listSolvers(cerr);

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(cout, true);
  }

  if (listRobustKernels) {
    std::vector<std::string> kernels;
    RobustKernelFactory::instance()->fillKnownKernels(kernels);
    cout << "Robust Kernels:" << endl;
    for (size_t i = 0; i < kernels.size(); ++i) {
      cout << kernels[i] << endl;
    }
  }

  AbstractRobustKernelCreator* kernelCreator=0;
  if (robustKernel.size() > 0) {
    kernelCreator = RobustKernelFactory::instance()->creator(robustKernel);
  }

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

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


  OptimizableGraph::EdgeSet originalEdges=optimizer.edges();

  if (0 && outputfilename.size() > 0) {
    cerr << "saving " << outputfilename << " ... ";
    ofstream os(outputfilename.c_str());
    optimizer.save(os);
    cerr << "done." << endl;
    return 0;
  }

  EdgeCreator creator;
  creator.addAssociation("VERTEX_SE2;VERTEX_SE2;","EDGE_SE2");
  creator.addAssociation("VERTEX_SE2;VERTEX_XY;","EDGE_SE2_XY");
  creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_SE3:QUAT;","EDGE_SE3:QUAT");
  creator.addAssociation("VERTEX_SE3_NEW;VERTEX_SE3_NEW;","EDGE_SE3_NEW");

  Parameter* p0 = optimizer.parameter(0);
  if (p0){
    ParameterSE3Offset* originalParams = dynamic_cast<ParameterSE3Offset*>(p0);
    if (originalParams) {
      cerr << "ORIGINAL PARAMS" << endl;
      ParameterSE3Offset* se3OffsetParam = new ParameterSE3Offset();
      se3OffsetParam->setId(100);
      optimizer.addParameter(se3OffsetParam);
      std::vector<int> depthCamHParamsIds(1);
      depthCamHParamsIds[0]=se3OffsetParam->id();
      creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_TRACKXYZ;","EDGE_SE3_TRACKXYZ", depthCamHParamsIds);
    }
  }

  EdgeLabeler labeler(&optimizer);

  if (optimizer.vertices().size() == 0) {
    cerr << "Graph contains no vertices" << endl;
    return 1;
  }

  // allocating the desired solver + testing whether the solver is okay
  OptimizationAlgorithmProperty solverProperty, hsolverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct(strSolver, solverProperty);
  OptimizationAlgorithm* hsolver = solverFactory->construct(strHSolver, hsolverProperty);
  if (! solver) {
    cerr << "Error allocating solver. Allocating \"" << strSolver << "\" failed!" << endl;
    return 0;
  }
  if (! hsolver) {
    cerr << "Error allocating hsolver. Allocating \"" << strHSolver << "\" failed!" << endl;
    return 0;
  }

  set<int> vertexDimensions = optimizer.dimensions();
  if (! optimizer.isSolverSuitable(solverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given graph" << endl;
    return 3;
  }
  if (! optimizer.isSolverSuitable(hsolverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given graph" << endl;
    return 3;
  }

  optimizer.setAlgorithm(solver);

  int poseDim=*vertexDimensions.rbegin();
  string backboneVertexType;
  string backboneEdgeType;
  switch(poseDim){
  case 3:
    if (hierarchicalDiameter == -1)
      hierarchicalDiameter = 30;
    backboneEdgeType =   "EDGE_SE2";
    backboneVertexType = "VERTEX_SE2";
    if (uThreshold <0){
      uThreshold =1e-5;
    }
    break;
  case 6:
    if (hierarchicalDiameter == -1)
      hierarchicalDiameter = 4;
    backboneEdgeType =   "EDGE_SE3:QUAT";
    backboneVertexType = "VERTEX_SE3:QUAT";
    // if (useNewTypes){
    //   backboneEdgeType =   "EDGE_SE3_NEW";
    //   backboneVertexType = "VERTEX_SE3_NEW";
    // }

    if (uThreshold <0){
      uThreshold =1e-3;
    }
    break;
  default:
    cerr << "Fatal: unknown backbone type. The largest vertex dimension is: " << poseDim << "." << endl
	 <<"Exiting." << endl;
    return -1;
  }

  // here we need to chop the graph into many lil pieces


  // check for vertices to fix to remove DoF
  bool gaugeFreedom = optimizer.gaugeFreedom();
  OptimizableGraph::Vertex* gauge=optimizer.findGauge();




  if (gaugeFreedom) {
    if (! gauge) {
      cerr <<  "# cannot find a vertex to fix in this thing" << endl;
      return 2;
    } else {
      cerr << "# graph is fixed by node " << gauge->id() << endl;
      gauge->setFixed(true);
    }
  } else {
    cerr << "# graph is fixed by priors" << endl;
  }






  // sanity check
  HyperDijkstra d(&optimizer);
  UniformCostFunction f;
  d.shortestPaths(gauge,&f);
  //cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size()!=optimizer.vertices().size()) {
    cerr << CL_RED("Warning: d.visited().size() != optimizer.vertices().size()") << endl;
    cerr << "visited: " << d.visited().size() << endl;
    cerr << "vertices: " << optimizer.vertices().size() << endl;
  }


  // BATCH optimization

  optimizer.initializeOptimization();

  optimizer.computeActiveErrors();
  double loadChi=optimizer.activeChi2();

  cerr << "Initial chi2 = " << FIXED(loadChi) << endl;


  if (initialGuess)
    optimizer.computeInitialGuess();
  signal(SIGINT, sigquit_handler);

  optimizer.computeActiveErrors();
  double initChi = optimizer.activeChi2();

  //  if (robustKernel) {
  //cerr << "# Preparing robust error function ... ";
    for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
      if (kernelCreator) {
	e->setRobustKernel(kernelCreator->construct());
	if (huberWidth > 0)
	  e->robustKernel()->setDelta(huberWidth);
      }
    }
    //cerr << "done." << endl;
    //}
  optimizer.computeActiveErrors();


  StarSet stars;


  computeSimpleStars(stars, &optimizer, &labeler, &creator,
      gauge, backboneEdgeType, backboneVertexType, 0, hierarchicalDiameter,
      1, starIterations, uThreshold, debug);

  cerr << "stars computed, stars.size()= " << stars.size() << endl;

  cerr << "hierarchy done, determining border" << endl;
  EdgeStarMap hesmap;
  constructEdgeStarMap(hesmap, stars, false);
  computeBorder(stars, hesmap);

  OptimizableGraph::EdgeSet   eset;
  OptimizableGraph::VertexSet vset;
  OptimizableGraph::EdgeSet   heset;
  OptimizableGraph::VertexSet hvset;
  HyperGraph::VertexSet hgauge;
  for (StarSet::iterator it=stars.begin(); it!=stars.end(); ++it) {

    Star* s=*it;
    if (hgauge.empty())
      hgauge=s->gauge();

    for (HyperGraph::VertexSet::iterator git=s->gauge().begin();
	 git != s->gauge().end(); ++git) {
      hvset.insert(*git);
    }

    for (HyperGraph::EdgeSet::iterator iit=s->_starEdges.begin(); iit!=s->_starEdges.end(); ++iit){
      OptimizableGraph::Edge* e= (OptimizableGraph::Edge*) *iit;
      eset.insert(e);
      for (size_t i=0; i<e->vertices().size(); i++){
        vset.insert(e->vertices()[i]);
      }
    }
    for (HyperGraph::EdgeSet::iterator iit=s->starFrontierEdges().begin(); iit!=s->starFrontierEdges().end(); ++iit){
      OptimizableGraph::Edge* e= (OptimizableGraph::Edge*) *iit;
      heset.insert(e);
    }
  }
  cerr << "eset.size()= " << eset.size() << endl;
  cerr << "heset.size()= " << heset.size() << endl;

  ofstream starStream("stars.g2o");
  optimizer.saveSubset(starStream, eset);
  starStream.close();

  ofstream hstarStream("hstars.g2o");
  optimizer.saveSubset(hstarStream, heset);
  hstarStream.close();

  cerr << "stars done!" << endl;

  cerr << "optimizing the high layer" << endl;
  for (HyperGraph::VertexSet::iterator it = hgauge.begin(); it!=hgauge.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(true);
  }
  optimizer.setAlgorithm(hsolver);
  optimizer.initializeOptimization(heset);
  optimizer.setVerbose(true);
  if (initialGuess)
    optimizer.computeInitialGuess();

  optimizer.computeActiveErrors();
  double hInitChi = optimizer.activeChi2();

  optimizer.optimize(highIterations);

  optimizer.computeActiveErrors();
  double hFinalChi = optimizer.activeChi2();

  cerr << "done" << endl;


  if (! kernelCreator) {
    cerr << "# Robust error function disabled ";
    for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
      e->setRobustKernel(0);
    }
    cerr << "done." << endl;
  } else {
    cerr << "# Preparing robust error function ay low level done";
  }

  cerr << "fixing the hstructure, and optimizing the floating nodes" << endl;
  for (OptimizableGraph::VertexSet::iterator it = hvset.begin(); it!=hvset.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(true);
  }
  optimizer.initializeOptimization(eset);
  optimizer.computeInitialGuess();
  optimizer.optimize(1);
  cerr << "done" << endl;
  if (debug)  {
    ofstream os("debug_low_level.g2o");
    optimizer.saveSubset(os,eset);
  }


  cerr << "adding the original constraints, locking hierarchical solution and optimizing the free variables" << endl;
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(true);
  }
  for (HyperGraph::VertexSet::iterator it = hgauge.begin(); it!=hgauge.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(true);
  }
  optimizer.setAlgorithm(solver);
  optimizer.initializeOptimization(0);
  optimizer.computeInitialGuess();
  optimizer.optimize(lowIterations);


  cerr << "relaxing the full problem" << endl;
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(false);
  }
  for (HyperGraph::VertexSet::iterator it = hgauge.begin(); it!=hgauge.end(); ++it){
    OptimizableGraph::Vertex* g=dynamic_cast<OptimizableGraph::Vertex*>(*it);
    g->setFixed(true);
  }
  optimizer.setAlgorithm(solver);
  optimizer.initializeOptimization(0);
  int result = optimizer.optimize(lowIterations);
  if (result <0)
    cerr << "failure in low level optimization" << endl;

  optimizer.computeActiveErrors();
  double finalChi=optimizer.activeChi2();

  if  (summaryFile!="") {
    PropertyMap summary;
    summary.makeProperty<StringProperty>("filename", inputFilename);
    summary.makeProperty<IntProperty>("n_vertices", optimizer.vertices().size());

    int nLandmarks=0;
    int nPoses=0;
    int maxDim = *vertexDimensions.rbegin();
    for (HyperGraph::VertexIDMap::iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); ++it){
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() != maxDim) {
	nLandmarks++;
      } else
	nPoses++;
    }

    int nEdges=0;
    set<string> edgeTypes;
    for (HyperGraph::EdgeSet::iterator it=optimizer.edges().begin(); it!=optimizer.edges().end(); ++it){
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*> (*it);
      if (e->level()==0) {
	edgeTypes.insert(Factory::instance()->tag(e));
	nEdges++;
      }
    }
    stringstream edgeTypesString;
    for (std::set<string>::iterator it=edgeTypes.begin(); it!=edgeTypes.end(); ++it){
      edgeTypesString << *it << " ";
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


    ofstream os;
    os.open(summaryFile.c_str(), ios::app);
    summary.writeToCSV(os);
  }


  if (outputfilename.size() > 0) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      optimizer.saveSubset(cout,originalEdges);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      ofstream os(outputfilename.c_str());
      optimizer.saveSubset(os,originalEdges);
    }
    cerr << "done." << endl;
  }

  // destroy all the singletons
  //Factory::destroy();
  //OptimizationAlgorithmFactory::destroy();
  //HyperGraphActionLibrary::destroy();

  return 0;
}
