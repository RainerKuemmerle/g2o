//  ../aisnavigation-free/bin/g2o-unfold -i 30 -solver var_cholmod -v -gnudump vic.dat ../datasets/2D/victoriaPark/victoriaPark.g2o

#include <signal.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "g2o/types/slam2d/types_three_dof.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/types_six_dof_quat.h"
#include "g2o/types/sba/types_sba.h"

#include "g2o/core/estimate_propagator.h"

#include "g2o/core/sparse_optimizer.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"

#include "tools.h"


#include "g2o/core/block_solver.h"

//#ifdef GO_CHOLMOD_SUPPORT
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#endif
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

using namespace std;
using namespace g2o;


struct InvChi2CostFunction: public HyperDijkstra::CostFunction {
  virtual double operator ()(HyperGraph::Edge* edge, HyperGraph::Vertex* from, HyperGraph::Vertex* to);
};

double InvChi2CostFunction::operator () (HyperGraph::Edge* edge,
           HyperGraph::Vertex* from __attribute__((unused)), 
           HyperGraph::Vertex* to __attribute__((unused)) ) {
  
  OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
e->computeError();
if(e->robustKernel())
  e->robustifyError();
return 1.0/(1e-6 + e->chi2());
}


// sort according to max id, dimension, min id
struct IncrementalEdgesCompare {
  bool operator()(SparseOptimizer::Edge* const & e1, SparseOptimizer::Edge* const & e2)
  {
    const SparseOptimizer::Vertex* to1 = static_cast<const SparseOptimizer::Vertex*>(e1->vertex(0));
    const SparseOptimizer::Vertex* to2 = static_cast<const SparseOptimizer::Vertex*>(e2->vertex(0));

    int i11 = e1->vertex(0)->id(), i12 = e1->vertex(1)->id();
    if (i11 > i12){
      swap(i11, i12);
    }
    int i21 = e2->vertex(0)->id(), i22 = e2->vertex(1)->id();
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

// another fancy HACK, to see something in graphViewer
void updateDisplay(ostream& os, const SparseOptimizer& optimizer)
{
  // TODO edges missing
  // TODO other than 2D types missing
  for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer.vertices().begin();
       it != optimizer.vertices().end(); ++it) {
    const SparseOptimizer::Vertex* v = static_cast<const SparseOptimizer::Vertex*>(it->second);
    Eigen::Quaterniond quat;
    Eigen::Vector3d trans;

    bool writeVertex = false;
    bool writePoint  = false;

    const VertexSE2* v2se = dynamic_cast<const VertexSE2*>(v);
    if (v2se) {
      writeVertex = true;
      trans[0] = v2se->estimate().translation()[0];
      trans[1] = v2se->estimate().translation()[1];
      trans[2] = 0.0;
      static Eigen::Vector3d axis(0,0,1);
      quat = AngleAxis<double>(v2se->estimate().rotation().angle(), axis);
    }

    const VertexPointXY* v2point = dynamic_cast<const VertexPointXY*>(v);
    if (v2point) {
      writePoint = true;
      trans[0] = v2point->estimate()[0];
      trans[1] = v2point->estimate()[1];
      trans[2] = 0.0;
    }

    if (writeVertex) {
      os.put('V');
      int id = v->id();
      os.write((char*)&id, sizeof(int));
      os.write((char*)&trans.x(), sizeof(double));
      os.write((char*)&trans.y(), sizeof(double));
      os.write((char*)&trans.z(), sizeof(double));
      os.write((char*)&quat.w(), sizeof(double));
      os.write((char*)&quat.x(), sizeof(double));
      os.write((char*)&quat.y(), sizeof(double));
      os.write((char*)&quat.z(), sizeof(double));
    }

    if (writePoint) {
      os.put('P');
      os.write((char*)&trans.x(), sizeof(double));
      os.write((char*)&trans.y(), sizeof(double));
      os.write((char*)&trans.z(), sizeof(double));
    }
  }

  // done
  os << "F" << flush;
}

SparseOptimizer::Method str2method(const std::string& strMethod){
  if (strMethod=="Gauss") {
    cerr << "# Doing Gauss" << endl;
    return SparseOptimizer::GaussNewton;
  }
  if (strMethod=="Levenberg") {
    cerr << "# Doing Levenberg-Marquardt" << endl;
    return SparseOptimizer::LevenbergMarquardt;
  }
  cerr << "# Unknown optimization method: " << strMethod << ", setting  default to Levenberg"  << endl;
  return SparseOptimizer::LevenbergMarquardt;
}

template<typename T>
typename T::LinearSolverType* allocateLinearSolver(bool useCholmod)
{
  if (useCholmod) {
#ifdef GO_CHOLMOD_SUPPORT
    return new LinearSolverCholmod<typename T::PoseMatrixType>;
#else
    return 0;
#endif
  } else {
    return new LinearSolverCSparse<typename T::PoseMatrixType>;
  }
}

Solver* str2solver(const std::string& strSolver_, SparseOptimizer* opt)
{
  string strSolver = strSolver_;
  bool useCholmod = false;
  if (strEndsWith(strSolver, "_cholmod")) {
    string::size_type idx = strSolver.rfind("_cholmod");
    strSolver = strSolver.substr(0, idx);
    useCholmod = true;
  }
#ifndef GO_CHOLMOD_SUPPORT
  if (useCholmod) {
    cerr << "Error: Cholmod is not supported in this build" << endl;
    return 0;
  }
#endif
  if (! strStartsWith(strSolver, "pcg")) 
    cerr << "# Using " << (useCholmod ? "cholmod" : "CSparse") << " " << strSolver << endl;
  else
    cerr << "# Using PCG " << strSolver<< endl;

  if (strSolver=="var") {
    BlockSolverX::LinearSolverType* linearSolver = allocateLinearSolver<BlockSolverX>(useCholmod);
    return new BlockSolverX(opt, linearSolver);
  }
  if (strSolver=="fix3_2") {
    BlockSolver_3_2::LinearSolverType* linearSolver = allocateLinearSolver<BlockSolver_3_2>(useCholmod);
    return new BlockSolver_3_2(opt, linearSolver);
  }
  if (strSolver=="fix6_3") {
    BlockSolver_6_3::LinearSolverType* linearSolver = allocateLinearSolver<BlockSolver_6_3>(useCholmod);
    return new BlockSolver_6_3(opt, linearSolver);
  }
  if (strSolver=="fix7_3") {
    BlockSolver_7_3::LinearSolverType* linearSolver = allocateLinearSolver<BlockSolver_7_3>(useCholmod);
    return new BlockSolver_7_3(opt, linearSolver);
  }
  if (strSolver == "pcg") {
    BlockSolverX::LinearSolverType* linearSolver = new LinearSolverPCG<BlockSolverX::PoseMatrixType>();
    return new BlockSolverX(opt, linearSolver);
  }
  if (strSolver == "pcg6_3") {
    BlockSolver_6_3::LinearSolverType* linearSolver = new LinearSolverPCG<BlockSolver_6_3::PoseMatrixType>();
    return new BlockSolver_6_3(opt, linearSolver);
  }
  if (strSolver == "pcg3_2") {
    BlockSolver_3_2::LinearSolverType* linearSolver = new LinearSolverPCG<BlockSolver_3_2::PoseMatrixType>();
    return new BlockSolver_3_2(opt, linearSolver);
  }
  cerr << "Unknown solver, setting the general but slower \"var\" "<< endl;
  BlockSolverX::LinearSolverType* linearSolver = allocateLinearSolver<BlockSolverX>(useCholmod);
  return new BlockSolverX(opt, linearSolver);
}


void gnudump_edges(string gnudump, 
      string file_suffix, 
      HyperGraph::EdgeSet::const_iterator begin, 
      HyperGraph::EdgeSet::const_iterator end,
      bool dumpEdges,
      bool dumpFeatures) {

  // ------- 
  if (gnudump.size() > 0) {
    string baseName = getPureFilename(gnudump);
    string extension = getFileExtension(gnudump);
    string newFilename = formatString("%s_%s.%s", baseName.c_str(), file_suffix.c_str(), extension.c_str());

    cerr << "Gnudump (" << newFilename << ")...  ";


    ofstream fout(newFilename.c_str());
    
    for (HyperGraph::EdgeSet::const_iterator eit = begin; eit != end; ++eit) {
      
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(const_cast<HyperGraph::Edge*>(*eit));
      e->computeError();
      if(e->robustKernel())
        e->robustifyError();
      double edgeChi2 = e->chi2();

      const VertexSE2* from = dynamic_cast<const VertexSE2*>((*eit)->vertex(0));
      const VertexSE2* to   = dynamic_cast<const VertexSE2*>((*eit)->vertex(1));
   
      if(dumpEdges) { 
  if (from && to) {
    Vector3d p1 = from->estimate().toVector();
    Vector3d p2 = to->estimate().toVector();
    fout << p1[0] << " " << p1[1] << " " << p1[2] <<  " " << edgeChi2 << " " << endl;
    fout << p2[0] << " " << p2[1] << " " << p1[2] <<  " " << edgeChi2 << " " << endl;
    fout << endl;
    continue;
  }
      }

      if(dumpFeatures) {
  const VertexPointXY* tox  = dynamic_cast<const VertexPointXY*>((*eit)->vertex(1));
  if (from && tox) {
    Vector3d p1 = from->estimate().toVector();
    Vector2d p2 = tox->estimate();
    fout << p1[0] << " " << p1[1] << " " << p1[2] <<  " " << edgeChi2 << " " << endl;
    fout << p2[0] << " " << p2[1] << " " << edgeChi2 << " " << endl;
    fout << endl;
    continue;
  }
      }


    }
    fout.close();
    cerr << " done" << endl;
  }
}

void gnudump_features(string gnudump, 
          string file_suffix, 
          HyperGraph::EdgeSet::const_iterator begin, 
          HyperGraph::EdgeSet::const_iterator end) {

  // ------- 
  if (gnudump.size() > 0) {
    string baseName = getPureFilename(gnudump);
    string extension = getFileExtension(gnudump);
    string newFilename = formatString("%s_%s.%s", baseName.c_str(), file_suffix.c_str(), extension.c_str());

    cerr << "Gnudump (" << newFilename << ")...  ";


    ofstream fout(newFilename.c_str());
    
    for (HyperGraph::EdgeSet::const_iterator eit = begin; eit != end; ++eit) {
      
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*eit);
      e->computeError();
      if(e->robustKernel())
        e->robustifyError();
      double edgeChi2 = e->chi2();

      const VertexPointXY* tox  = dynamic_cast<const VertexPointXY*>((*eit)->vertex(1));
      if (tox) {
  Vector2d p2 = tox->estimate();
  fout << p2[0] << " " << p2[1] << " " << edgeChi2 << " " << endl;
  continue;
      }
    }
    fout.close();
    cerr << " done" << endl;
  }
}




bool hasToStop=false;

void sigquit_handler(int q __attribute__((unused))){
  hasToStop=1;
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
  bool incremental;
  bool guiOut;
  double lambdaInit;
  int updateGraphEachN = 10;
  string statsFile;
  // command line parsing
  CommandArgs arg;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("guess", initialGuess, false, "initial guess based on spanning tree");
  arg.param("inc", incremental, false, "run incremetally");
  arg.param("update", updateGraphEachN, 10, "updates afert x odometry nodes, (default: 10)");
  arg.param("guiOut", guiOut, false, "gui output while running incrementally");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("marginalize", marginalize, false, "on or off");
  arg.param("method", strMethod, "Gauss", "Gauss or Levenberg");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "var", "specify which solver to use underneat\n\t {var, fix3_2, fix6_3, fix_7_3}");
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("renameTypes", loadLookup, "", "create a lookup for loading types into other types,\n\t TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");

  arg.parseArgs(argc, argv);

  if (listTypes) {
    OptimizableGraph::printRegisteredTypes(cout, true);
  }

  SparseOptimizer optimizer;
  optimizer.verbose() = verbose;
  optimizer.setForceStopFlag(&hasToStop);
  
  optimizer.method() = str2method(strMethod);
  optimizer.solver() = str2solver(strSolver, &optimizer);
  if (! optimizer.solver()) {
    cerr << "Error allocating solver" << endl;
    return 0;
  }
  optimizer.userLambdaInit()=lambdaInit;

  assert (optimizer.solver());
  ActivePathCostFunction* guessCostFunction=0;
  if (initialGuess)
    guessCostFunction=new ActivePathCostFunction(&optimizer);

  cerr << "Read input from " << inputFilename << endl;
  ifstream ifs(inputFilename.c_str());
  if (!ifs) {
    cerr << "Failed to open file" << endl;
    return 1;
  }

  if (loadLookup.size() > 0) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }

  if (!optimizer.load(ifs)) {
    cerr << "Error loading graph" << endl;
    return 2;
  }

  cerr << "Loaded " << optimizer.vertices().size() << " vertices" << endl;
  cerr << "Loaded " << optimizer.edges().size() << " edges" << endl;

  if (optimizer.vertices().size() == 0) {
    cerr << "HyperGraph contains no vertices" << endl;
    return 1;
  }

  // HACK if schur, we wanna marginalize the landmarks...
  if (marginalize)
    {
      cerr << "Preparing Marginalization of the Landmarks ... ";
      int maxDim = -1;
      for (HyperGraph::VertexIDMap::iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); it++){
  OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
  maxDim = (max)(v->dimension(), maxDim);
      }
      for (HyperGraph::VertexIDMap::iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); it++){
  OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
  if (v->dimension() != maxDim) {
    //cerr << "m";
    v->marginalized() = true;
  }
      }
      cerr << "done." << endl;
    }

  // sanity check
  //HyperDijkstra d(&optimizer);
  //UniformCostFunction f;
  //d.shortestPaths(optimizer.findOneOrigin(),&f);
  //cerr << PVAR(d.visited().size()) << endl;
  //assert (d.visited().size()==optimizer.vertices().size());


  if (incremental) {
    int incIterations = 1;
    int updateDisplayEveryN = updateGraphEachN;
    int maxDim = 0;

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
    optimizer.verbose() = false;

    // sort the edges in a way that inserting them makes sense
    sort(edges.begin(), edges.end(), IncrementalEdgesCompare());

    double cumTime = 0.;
    int vertexCount=0;
    int lastOptimizedVertexCount = 0;
    int lastVisUpdateVertexCount = 0;
    bool addNextEdge=true;
    bool freshlyOptimized=false;
    for (vector<SparseOptimizer::Edge*>::iterator it = edges.begin(); it != edges.end(); ++it) {
      SparseOptimizer::Edge* e = *it;
      bool optimize=false;

      if (addNextEdge && !optimizer.vertices().empty()){
        int maxInGraph = optimizer.vertices().rbegin()->first;
        int idMax = max(e->vertex(0)->id(), e->vertex(1)->id());
        if (maxInGraph < idMax && ! freshlyOptimized){
          addNextEdge=false;
          optimize=true;
        } else {
          addNextEdge=true;
          optimize=false;
        }
      }

      int doInit = 0;
      SparseOptimizer::Vertex* v1 = optimizer.vertex(e->vertex(0)->id());
      SparseOptimizer::Vertex* v2 = optimizer.vertex(e->vertex(1)->id());
      if (! v1 && addNextEdge) {
        //cerr << " adding vertex " << it->id1 << endl;
        SparseOptimizer::Vertex* v = dynamic_cast<SparseOptimizer::Vertex*>(e->vertex(0));
        bool v1Added = optimizer.addVertex(v);
        assert(v1Added);
        doInit = 2;
        if (v->dimension() == maxDim)
          vertexCount++;
      }

      if (! v2 && addNextEdge) {
        //cerr << " adding vertex " << it->id2 << endl;
        SparseOptimizer::Vertex* v = dynamic_cast<SparseOptimizer::Vertex*>(e->vertex(1));
        bool v2Added = optimizer.addVertex(v);
        assert(v2Added);
        doInit = 1;
        if (v->dimension() == maxDim)
          vertexCount++;
      }

      if (addNextEdge){
        //cerr << " adding edge " << it->id1 <<  " " << it->id2 << " " << it->mean << endl;
        if (! optimizer.addEdge(e)) {
          cerr << "Unable to add edge " << e->vertex(0)->id() << " -> " << e->vertex(1)->id() << endl;
        }

        if (doInit) {
          OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(e->vertex(0));
          OptimizableGraph::Vertex* to   = static_cast<OptimizableGraph::Vertex*>(e->vertex(1));
          switch (doInit){
            case 1: // initialize v1 from v2
              if (e->initialEstimatePossible(to, from)) {
                // cerr << "init: " 
                //      << to->id() << "(" << to->dimension() << ") -> " 
                //      << from->id() << "(" << from->dimension() << ") " << endl;
                e->initialEstimate(to, from);
              }
              break;
            case 2: 
              if (e->initialEstimatePossible(from, to)) {
                // cerr << "init: " 
                //      << from->id() << "(" << from->dimension() << ") -> " 
                //      << to->id() << "(" << to->dimension() << ") " << endl;
                e->initialEstimate(from, to);  
              }
              break;
            default: cerr << "doInit wrong value\n"; 
          }
        }

      }

      freshlyOptimized=false;
      if (optimize){
        //cerr << "Optimize" << endl;
        if (vertexCount - lastOptimizedVertexCount >= updateGraphEachN) {
          double ts = get_monotonic_time();
          int currentIt=optimizer.optimize(incIterations);
          double dts = get_monotonic_time() - ts;
          cumTime += dts;
          //optimizer->setOptimizationTime(cumTime);
          if (verbose) {
            double chi2 = optimizer.chi2();
            cerr << "nodes= " << optimizer.vertices().size() << "\t edges= " << optimizer.edges().size() << "\t chi2= " << chi2
     << "\t time= " << dts << "\t iterations= " << currentIt <<  "\t cumTime= " << cumTime << endl;
          }
          lastOptimizedVertexCount = vertexCount;
        }

        if (! verbose)
          cerr << ".";
        addNextEdge=true;
        freshlyOptimized=true;
        it--;
      }

      if (guiOut) {
        if (vertexCount - lastVisUpdateVertexCount >= updateDisplayEveryN) {
          updateDisplay(cout, optimizer);
          lastVisUpdateVertexCount = vertexCount;
        }
      }

    } // for all edges


  } else {

    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "inputtraj", optimizer.edges().begin(), optimizer.edges().end(), true, false);
  if (gnudump.size() > 0) 
    gnudump_features(gnudump, "inputfeatures", optimizer.edges().begin(), optimizer.edges().end());
    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "input", optimizer.edges().begin(), optimizer.edges().end(), true, true);

    // BATCH optimization
    //    signal(SIGINT, sigquit_handler);
    cerr << "Initial chi2 = " << optimizer.chi2() << endl;

    if (statsFile!=""){
      // allocate buffer for statistics;
      optimizer._statistics=new G2OBatchStatistics[maxIterations];
    }

    int i=optimizer.optimize(maxIterations,guessCostFunction);
    if (!i){
      cerr << "Cholesky failed, result might be invalid" << endl;
    }

    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "regularopttraj", optimizer.edges().begin(), optimizer.edges().end(), true, false);
  if (gnudump.size() > 0) 
    gnudump_features(gnudump, "regularoptfeatures", optimizer.edges().begin(), optimizer.edges().end());
    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "regularopt", optimizer.edges().begin(), optimizer.edges().end(), true, true);
    

    cerr << "Computing chi2 statistics ";
    double sumChi2=0.0;
    double chi2Thres=0.0;
    int cnt=0;
  
    for (HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->computeError();
      if(e->robustKernel())
        e->robustifyError();
      sumChi2 += e->chi2();
      cnt++;
    }
    chi2Thres = sumChi2/cnt * 1.4;
    cerr << "  threshold=" << chi2Thres << "  done" << endl;

    cerr << "Searchin for high error edges .. ";
    HyperGraph::EdgeSet highErrorEdges;
    HyperGraph::EdgeSet highErrorEdgesToFeatures;
    HyperGraph::EdgeSet edgesToOptimize;

    HyperGraph::EdgeSet edgesToOptimize_selected;
    HyperGraph::EdgeSet edgesToOptimize_border;
  
    for (HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->computeError();
      if(e->robustKernel())
        e->robustifyError();
      double edgeChi2 = e->chi2();
      if (edgeChi2 > chi2Thres) {

  const VertexSE2* from = dynamic_cast<const VertexSE2*>(e->vertex(0));
  const VertexSE2* to   = dynamic_cast<const VertexSE2*>(e->vertex(1));

  if (from && to) {
    highErrorEdges.insert(*it);
    continue;
  }
  const VertexPointXY* tox  = dynamic_cast<const VertexPointXY*>(e->vertex(1));
  if (from && tox) {
    highErrorEdgesToFeatures.insert(*it);
    continue;
  }
      }
    }
    cerr << " found=" << highErrorEdges.size() << "/" << highErrorEdgesToFeatures.size() << " edges with high errors .. done" << endl;

  
    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "hee", highErrorEdges.begin(), highErrorEdges.end(), true, false);

    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "heef_", highErrorEdgesToFeatures.begin(), highErrorEdgesToFeatures.end(), true, true);
  

    // fix all vertices
    cerr << "Fixing the whole graph ";
    for (std::map<int, HyperGraph::Vertex*>::iterator vit = optimizer.vertices().begin(); 
   vit != optimizer.vertices().end(); ++vit) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>((*vit).second);
      v->fixed() = true;
    }
    cerr << " done" << endl;

    typedef std::pair< HyperGraph::EdgeSet, HyperGraph::EdgeSet > EdgeSetPair;
    typedef std::vector< EdgeSetPair > EdgeSetPairVector;

    EdgeSetPairVector myEdgeStack;

    cerr << "Collecting clusters...";

    while (!highErrorEdges.empty()) {

      HyperGraph::EdgeSet::iterator it = highErrorEdges.begin(); 

      HyperGraph::EdgeSet selected;    
      HyperGraph::EdgeSet border;    
      InvChi2CostFunction c2cost;

      HyperGraph::Edge* start = *it;
      bool edgesSelected = false;
      
      findConnectedEdgesWithCostLimit(selected, border, start, &c2cost, 2.0/chi2Thres);

 
      if (selected.size() > 10)  {
  edgesSelected = true;
  cerr << " (" << selected.size() << ", " << border.size() << ")";

  // high error edges and their neighbor edges will be considered for optimization

  edgesToOptimize_selected.insert(selected.begin(), selected.end());
  edgesToOptimize_border.insert(border.begin(), border.end());
  edgesToOptimize.insert(selected.begin(), selected.end());
  edgesToOptimize.insert(border.begin(), border.end());
      }
      else {
  cerr << " [" << selected.size() << ", " << border.size() << "]";
      }

      // removed edges from that cluster from the open list of high error edges
      for (HyperGraph::EdgeSet::iterator eit = selected.begin(); eit != selected.end(); ++eit) {
  HyperGraph::EdgeSet::iterator removeMe = highErrorEdges.find(*eit);
  if (removeMe != highErrorEdges.end()) {
    highErrorEdges.erase(removeMe);
  }
      }
      
      if (edgesSelected)  {
  myEdgeStack.push_back( EdgeSetPair(selected,border) );
      }
    }
    cerr << " done" << endl;

    cerr << "Found " << myEdgeStack.size() 
   << " clusters in sum with " << edgesToOptimize_selected.size() << " high error edges" 
   << " and " << edgesToOptimize_border.size() << " border edges." << endl;


    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "selected", edgesToOptimize_selected.begin(), edgesToOptimize_selected.end(),true, false);

    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "border", edgesToOptimize_border.begin(), edgesToOptimize_border.end(),true, false);

    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "reoptimizeme", edgesToOptimize.begin(), edgesToOptimize.end(),true, false);

    cerr << "Unfixing high error edges ";
    for (EdgeSetPairVector::iterator it = myEdgeStack.begin(); 
   it != myEdgeStack.end(); ++it) {

      // un-fix bad vertices
      for (HyperGraph::EdgeSet::iterator eit = (*it).first.begin(); eit !=  (*it).first.end(); ++eit) {
  OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*eit);

  OptimizableGraph::Vertex* vfrom = static_cast<OptimizableGraph::Vertex*>(e->vertex(0));
  vfrom->fixed() = false;
  OptimizableGraph::Vertex* vto = static_cast<OptimizableGraph::Vertex*>(e->vertex(1));
  vto->fixed() = false;
      }
    }
    cerr << "done" << endl;


    // generate that for all nodes in all clusters
    cerr << "Initialize optimization of subgraphs .. ";
    optimizer.initializeOptimization(edgesToOptimize);
    cerr << "done" << endl;

    if (guessCostFunction==0)
      guessCostFunction=new ActivePathCostFunction(&optimizer);


    cerr << "Searching for init nodes and re-initialize subgraphs .. ";
    for (EdgeSetPairVector::iterator it = myEdgeStack.begin(); 
   it != myEdgeStack.end(); ++it) {

      // determine good node for initialization  (one out of the borders!)
      cerr << "S";
      HyperGraph::EdgeSet::iterator eit = (*it).second.begin();
    
      OptimizableGraph::Vertex* initNode = 0;
      OptimizableGraph::Vertex* vfrom = static_cast<OptimizableGraph::Vertex*>((*eit)->vertex(0));
      OptimizableGraph::Vertex* vto = static_cast<OptimizableGraph::Vertex*>((*eit)->vertex(1));

      if (vfrom && vfrom->fixed())
  initNode = vfrom;
      else if (vto && vto->fixed())
  initNode = vto;
      else 
  initNode = vfrom;
    
      // reinitialize bad vertices
      cerr << "I";
      optimizer.initializeActiveSubsetViaMeasurements(initNode, guessCostFunction);
      cerr << "-";
    }      
    cerr << " .. done" << endl;
      
    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "subsetinitialized", optimizer.edges().begin(), optimizer.edges().end(),true, false);

  
    // run all local optimization
    cerr << "Optimize subgraphs .. "<< endl;
    optimizer.optimizeLoop(maxIterations);    
    cerr << "done" << endl;
  
    if (gnudump.size() > 0) 
      gnudump_edges(gnudump, "subsetoptimized", optimizer.edges().begin(), optimizer.edges().end(),true, false);
 

    if (0) // no effect visible, why??
    if (highErrorEdgesToFeatures.size() > 0) {

      HyperGraph::EdgeSet edgesToBadFeatures;

      // fix all vertices
      cerr << "Fixing the whole graph ";
      for (std::map<int, HyperGraph::Vertex*>::iterator vit = optimizer.vertices().begin(); 
     vit != optimizer.vertices().end(); ++vit) {
  OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>((*vit).second);
  v->fixed() = true;
      }
      cerr << " done" << endl;

      cerr << "Unfixing bad features";
      for (HyperGraph::EdgeSet::iterator eit = highErrorEdgesToFeatures.begin(); 
     eit != highErrorEdgesToFeatures.end(); ++eit) {

  OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>((*eit)->vertex(1));
  v->fixed() = false;

  edgesToBadFeatures.insert(v->edges().begin(), v->edges().end());
      } 

      cerr << "  considering " << edgesToBadFeatures.size() << " edges .. done"<< endl;

      if (gnudump.size() > 0) 
  gnudump_edges(gnudump, "featurereinit", edgesToBadFeatures.begin(), edgesToBadFeatures.end(), true, true);
      if (gnudump.size() > 0) 
  gnudump_features(gnudump, "featurereinitfeatures", edgesToBadFeatures.begin(), edgesToBadFeatures.end());


      cerr << "Initialize optimization of bad features .. ";
      optimizer.initializeOptimization(edgesToBadFeatures);
      cerr << "done" << endl;


      // run all local optimization
      cerr << "Optimize bad features .. "<< endl;
      optimizer.optimizeLoop(maxIterations/2);    
      cerr << "done" << endl;

      if (gnudump.size() > 0) 
  gnudump_edges(gnudump, "subgraphsfinal", optimizer.edges().begin(), optimizer.edges().end(),true, false);
    }
    
  
    // un-fix all vertices
    for (std::map<int, HyperGraph::Vertex*>::iterator vit = optimizer.vertices().begin(); 
   vit != optimizer.vertices().end(); ++vit) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>((*vit).second);
      v->fixed() = false;
    }

    // final global optimization
    cerr << "Initialize final optimization .. ";
    optimizer.initializeOptimization(optimizer.edges());// ALLLL
    cerr << "done" << endl;
    cerr << "Optimize .. " << endl;
    optimizer.optimizeLoop(maxIterations);    
    cerr << "done" << endl;
  
  }
  

  if (gnudump.size() > 0) 
    gnudump_edges(gnudump, "finaltraj", optimizer.edges().begin(), optimizer.edges().end(),true,false);
  if (gnudump.size() > 0) 
    gnudump_features(gnudump, "finalfeatures", optimizer.edges().begin(), optimizer.edges().end());
  if (gnudump.size() > 0) 
    gnudump_edges(gnudump, "final", optimizer.edges().begin(), optimizer.edges().end(),true,true);

  
  if (statsFile!=""){
    cerr << "writing stats to file \"" << statsFile << "\"";
    ofstream os(statsFile.c_str());
    for (int i=0; i<maxIterations; i++){
      os << optimizer._statistics[i] << endl;
    }
  }

   

  //     // HACK write landmarks to anotherfile
  // #   if 1
  //     bool hasLandmarks = false;
  //     for (HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
  //       const VertexSE2* from = dynamic_cast<const VertexSE2*>((*it)->vertex(0]);
  //       const VertexPointXY* to = dynamic_cast<const VertexPointXY*>((*it)->vertex(1]);
  //       if (from && to) {
  //         hasLandmarks = true;
  //         break;
  //       }
  //     }
  //     if (hasLandmarks) {
  //       string baseName = getPureFilename(gnudump);
  //       string extension = getFileExtension(gnudump);
  //       string landmarkFilename = formatString("%s_landmarks.%s", baseName.c_str(), extension.c_str());
  //       cerr << "saving " << landmarkFilename << " ... ";
  //       fout.open(landmarkFilename.c_str());
  //       for (HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
  //         const VertexPointXY* to = dynamic_cast<const VertexPointXY*>((*it)->vertex(1]);
  //         if (to) {
  //           const Vector2d& p2 = to->estimate();
  //           fout << p2[0] << " " << p2[1] << endl;
  //           fout << endl;
  //         }
  //       }
  //       cerr << "done." << endl;
  //     }
  // #   endif

  //}

  if (outputfilename.size() > 0) {
    cerr << "saving " << outputfilename << " ... ";
    optimizer.save(outputfilename.c_str());
    cerr << "done." << endl;
  }

  return 0;
}
