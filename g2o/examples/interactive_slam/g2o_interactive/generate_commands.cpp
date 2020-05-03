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

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"

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
  string loadLookup;
  bool listTypes;
  int updateGraphEachN = 10;
  string dummy;
  // command line parsing
  CommandArgs arg;
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes, (default: 10)");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("renameTypes", loadLookup, "", "create a lookup for loading types into other types,\n\t TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(cout, true);
  }

  SparseOptimizer optimizer;

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
    int maxDim = 0;

    cerr << "# incremental setttings" << endl;
    cerr << "#\t solve every " << updateGraphEachN << endl;

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

      SparseOptimizer::Vertex* v1 = optimizer.vertex(e->vertices()[0]->id());
      SparseOptimizer::Vertex* v2 = optimizer.vertex(e->vertices()[1]->id());
      if (! v1 && addNextEdge) {
        //cerr << " adding vertex " << it->id1 << endl;
        SparseOptimizer::Vertex* v = dynamic_cast<SparseOptimizer::Vertex*>(e->vertices()[0]);
        bool v1Added = optimizer.addVertex(v);
        maxInGraph = (max)(maxInGraph, v->id());
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v1Added);
        if (! v1Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
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
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v2Added);
        if (! v2Added)
          cerr << "Error adding vertex " << v->id() << endl;
        else
          verticesAdded.insert(v);
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
        if (e->dimension() == 3) {
          static int edgeCnt = 0;
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
        --it;
      }

    } // for all edges

  }

  return 0;
}
