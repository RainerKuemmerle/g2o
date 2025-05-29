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
#include <iostream>
#include <memory>
#include <string>

#include "CLI/CLI.hpp"
#include "g2o/core/factory.h"
#include "g2o/core/sparse_optimizer.h"

namespace {

// sort according to max id, dimension, min id
struct IncrementalEdgesCompare {
  bool operator()(std::shared_ptr<g2o::SparseOptimizer::Edge> const& e1,
                  std::shared_ptr<g2o::SparseOptimizer::Edge> const& e2) {
    const auto* to1 = static_cast<const g2o::SparseOptimizer::Vertex*>(
        e1->vertices()[0].get());
    const auto* to2 = static_cast<const g2o::SparseOptimizer::Vertex*>(
        e2->vertices()[0].get());

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
    if (to1->dimension() !=
        to2->dimension()) {  // push the odometry to be the first
      return to1->dimension() > to2->dimension();
    }
    return (i11 < i21);
  }
};
}  // namespace

int main(int argc, char** argv) {
  std::string inputFilename;
  std::string loadLookup;
  bool listTypes;
  int updateGraphEachN = 10;
  // command line parsing
  CLI::App app{"g2o Generate Incremental Commands"};
  argv = app.ensure_utf8(argv);
  app.add_option("--update", updateGraphEachN, "updates after x odometry nodes")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  app.add_flag("--list_types", listTypes, "list the registered types");
  app.add_option("--rename_types", loadLookup,
                 "create a lookup for loading types into other types,\n\t "
                 "TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., "
                 "VERTEX_CAM=VERTEX_SE3:EXPMAP");
  app.add_option("graph-input", inputFilename,
                 "graph file which will be processed ('-' for stdin)")
      ->required()
      ->check(CLI::ExistingFile);

  CLI11_PARSE(app, argc, argv);

  if (listTypes) {
    g2o::Factory::instance()->printRegisteredTypes(std::cout, true);
  }

  g2o::SparseOptimizer optimizer;

  // Loading the input data
  if (loadLookup.size() > 0) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.empty()) {
    std::cerr << "No input data specified\n";
    return 0;
  }

  if (inputFilename == "-") {
    std::cerr << "Read input from stdin\n";
    if (!optimizer.load(std::cin)) {
      std::cerr << "Error loading graph\n";
      return 2;
    }
  } else {
    std::cerr << "Read input from " << inputFilename << '\n';
    std::ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      std::cerr << "Failed to open file\n";
      return 1;
    }
    if (!optimizer.load(ifs)) {
      std::cerr << "Error loading graph\n";
      return 2;
    }
  }
  std::cerr << "Loaded " << optimizer.vertices().size() << " vertices\n";
  std::cerr << "Loaded " << optimizer.edges().size() << " edges\n";

  if (optimizer.vertices().size() == 0) {
    std::cerr << "Graph contains no vertices\n";
    return 1;
  }

  {
    int maxDim = 0;

    std::cerr << "# incremental settings\n";
    std::cerr << "#\t solve every " << updateGraphEachN << '\n';

    g2o::SparseOptimizer::VertexIDMap vertices = optimizer.vertices();
    for (auto& vertex : vertices) {
      const auto* v =
          static_cast<const g2o::SparseOptimizer::Vertex*>(vertex.second.get());
      maxDim = (std::max)(maxDim, v->dimension());
    }

    std::vector<std::shared_ptr<g2o::SparseOptimizer::Edge>> edges;
    for (const auto& edge : optimizer.edges()) {
      auto e = std::dynamic_pointer_cast<g2o::SparseOptimizer::Edge>(edge);
      if (e) edges.emplace_back(std::move(e));
    }
    optimizer.edges().clear();
    optimizer.vertices().clear();
    optimizer.setVerbose(false);

    // sort the edges in a way that inserting them makes sense
    sort(edges.begin(), edges.end(), IncrementalEdgesCompare());

    int vertexCount = 0;
    int lastOptimizedVertexCount = 0;
    bool addNextEdge = true;
    bool freshlyOptimized = false;
    g2o::HyperGraph::VertexSet verticesAdded;
    int maxInGraph = -1;
    for (auto it = edges.begin(); it != edges.end(); ++it) {
      const std::shared_ptr<g2o::SparseOptimizer::Edge>& e = *it;
      bool optimize = false;

      if (addNextEdge && !optimizer.vertices().empty()) {
        int idMax = (std::max)(e->vertices()[0]->id(), e->vertices()[1]->id());
        if (maxInGraph < idMax && !freshlyOptimized) {
          addNextEdge = false;
          optimize = true;
        } else {
          addNextEdge = true;
          optimize = false;
        }
      }

      std::shared_ptr<g2o::SparseOptimizer::Vertex> v1 =
          optimizer.vertex(e->vertices()[0]->id());
      std::shared_ptr<g2o::SparseOptimizer::Vertex> v2 =
          optimizer.vertex(e->vertices()[1]->id());
      if (!v1 && addNextEdge) {
        // cerr << " adding vertex " << it->id1 << endl;
        auto v = std::dynamic_pointer_cast<g2o::SparseOptimizer::Vertex>(
            e->vertices()[0]);
        bool v1Added = optimizer.addVertex(v);
        maxInGraph = (std::max)(maxInGraph, v->id());
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" <<
        // endl;
        assert(v1Added);
        if (!v1Added)
          std::cerr << "Error adding vertex " << v->id() << '\n';
        else
          verticesAdded.insert(v);
        if (v->dimension() == maxDim) vertexCount++;

        if (v->dimension() == 3) {
          std::cout << "ADD VERTEX_XYT " << v->id() << ";\n";
        } else if (v->dimension() == 6) {
          std::cout << "ADD VERTEX_XYZRPY " << v->id() << ";\n";
        }
      }

      if (!v2 && addNextEdge) {
        auto v = std::dynamic_pointer_cast<g2o::SparseOptimizer::Vertex>(
            e->vertices()[1]);
        // cerr << " adding vertex " << v->id() << endl;
        bool v2Added = optimizer.addVertex(v);
        maxInGraph = (std::max)(maxInGraph, v->id());
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" <<
        // endl;
        assert(v2Added);
        if (!v2Added)
          std::cerr << "Error adding vertex " << v->id() << '\n';
        else
          verticesAdded.insert(v);
        if (v->dimension() == maxDim) vertexCount++;

        if (v->dimension() == 3) {
          std::cout << "ADD VERTEX_XYT " << v->id() << ";\n";
        } else if (v->dimension() == 6) {
          std::cout << "ADD VERTEX_XYZRPY " << v->id() << ";\n";
        }
      }

      if (addNextEdge) {
        if (e->dimension() == 3) {
          static int edgeCnt = 0;
          double* information = e->informationData();
          double meas[3];
          e->getMeasurementData(meas);
          // ADD EDGE_XYT 1 1 2 .1 .2 .3 1 0 0 1 0 1;
          std::cout << "ADD EDGE_XYT " << edgeCnt++ << " "
                    << e->vertices()[0]->id() << " " << e->vertices()[1]->id()
                    << " " << meas[0] << " " << meas[1] << " " << meas[2];
          for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j)
              std::cout << " " << information[(i * 3) + j];
          std::cout << ";\n";
        } else if (e->dimension() == 6) {
          // TODO(Rainer) convert to EULER angles
          std::cerr << "NOT IMPLEMENTED YET\n";
        }
        static bool firstEdge = true;
        if (firstEdge) {
          firstEdge = false;
          std::cout << "FIX 0;\n";
        }

        // cerr << " adding edge " << e->vertices()[0]->id() <<  " " <<
        // e->vertices()[1]->id() << endl;
        if (!optimizer.addEdge(e)) {
          std::cerr << "Unable to add edge " << e->vertices()[0]->id() << " -> "
                    << e->vertices()[1]->id() << '\n';
        }
      }

      freshlyOptimized = false;
      if (optimize) {
        // cerr << "Optimize" << endl;
        if (vertexCount - lastOptimizedVertexCount >= updateGraphEachN) {
          std::cout << "SOLVE_STATE;\n";
          std::cout << "QUERY_STATE;\n";
          lastOptimizedVertexCount = vertexCount;
        }

        addNextEdge = true;
        freshlyOptimized = true;
        --it;
      }

    }  // for all edges
  }

  return 0;
}
