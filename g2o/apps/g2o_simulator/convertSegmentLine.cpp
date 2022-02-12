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

#include "g2o/apps/g2o_simulator/simutils.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

using std::cerr;
using std::endl;
using std::string;

namespace g2o {

struct LineInfo {
  explicit LineInfo(VertexSegment2D* s) {
    line = std::make_shared<VertexLine2D>();
    line->setId(s->id());
    line->setEstimate(
        Line2D(computeLineParameters(s->estimateP1(), s->estimateP2())));
  }
  std::shared_ptr<VertexLine2D> line;
  std::shared_ptr<VertexPointXY> p1;
  std::shared_ptr<VertexPointXY> p2;
};

using LineInfoMap = std::map<int, LineInfo>;

static int run_main(int argc, char** argv) {
  string outputfilename;
  string inputFilename;
  CommandArgs arg;
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed", true);

  arg.parseArgs(argc, argv);
  OptimizableGraph inGraph;

  // registering all the types from the libraries

  if (inputFilename.empty()) {
    cerr << "No input data specified" << endl;
    return 0;
  }
  if (inputFilename == "-") {
    cerr << "Read input from stdin" << endl;
    if (!inGraph.load(std::cin)) {
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
    if (!inGraph.load(ifs)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  }
  cerr << "Loaded " << inGraph.vertices().size() << " vertices" << endl;
  cerr << "Loaded " << inGraph.edges().size() << " edges" << endl;

  cerr << "filling in linfoMap and odoms" << endl;
  LineInfoMap lmap;
  OptimizableGraph outGraph;
  // insert all lines in the infomap
  int currentId = -1000;
  bool firstVertexFound = false;
  for (auto& it : inGraph.vertices()) {
    currentId = currentId > it.first ? currentId : it.first;

    auto* pose = dynamic_cast<VertexSE2*>(it.second.get());
    if (pose) {
      auto npose = std::make_shared<VertexSE2>();
      npose->setEstimate(pose->estimate());
      npose->setId(pose->id());
      outGraph.addVertex(npose);
      if (!firstVertexFound) {
        firstVertexFound = true;
        npose->setFixed(true);
      }
    }

    auto* s = dynamic_cast<VertexSegment2D*>(it.second.get());
    if (s) {
      LineInfo linfo(s);
      outGraph.addVertex(linfo.line);
      lmap.insert(std::make_pair(s->id(), linfo));
    }
  }

  currentId++;

  cerr << "filling in edges and odoms" << endl;
  for (const auto& it : inGraph.edges()) {
    auto* ods = dynamic_cast<EdgeSE2*>(it.get());
    if (ods) {
      auto ods2 = std::make_shared<EdgeSE2>();
      ods2->setMeasurement(ods->measurement());
      ods2->setInformation(ods->information());
      ods2->vertices()[0] = outGraph.vertex(ods->vertices()[0]->id());
      ods2->vertices()[1] = outGraph.vertex(ods->vertices()[1]->id());
      outGraph.addEdge(ods2);
    }

    auto* es = dynamic_cast<EdgeSE2Segment2D*>(it.get());
    auto* el = dynamic_cast<EdgeSE2Segment2DLine*>(it.get());
    auto* espl = dynamic_cast<EdgeSE2Segment2DPointLine*>(it.get());

    if (es || el || espl) {
      auto pose = std::dynamic_pointer_cast<VertexSE2>(it->vertices()[0]);
      auto segment =
          std::dynamic_pointer_cast<VertexSegment2D>(it->vertices()[1]);
      if (!pose) continue;
      pose = std::dynamic_pointer_cast<VertexSE2>(outGraph.vertex(pose->id()));
      auto lit = lmap.find(segment->id());
      assert(lit != lmap.end());
      LineInfo& linfo = lit->second;
      auto line = linfo.line;
      auto& p1 = linfo.p1;
      auto& p2 = linfo.p2;

      auto el2 = std::make_shared<EdgeSE2Line2D>();
      el2->vertices()[0] = pose;
      el2->vertices()[1] = line;
      if (el) {
        el2->setMeasurement(Line2D(el->measurement()));
        el2->setInformation(el->information());
        outGraph.addEdge(el2);
      }
      if (es) {
        el2->setMeasurement(Line2D(
            computeLineParameters(es->measurementP1(), es->measurementP2())));
        Matrix2 el2info;
        el2info << 10000, 0, 0, 1000;
        el2->setInformation(el2info);
        outGraph.addEdge(el2);
        Matrix4 si = es->information();
        if (!p1) {
          p1 = std::make_shared<VertexPointXY>();
          p1->setEstimate(segment->estimateP1());
          p1->setId(currentId++);
          outGraph.addVertex(p1);
          line->p1Id = p1->id();

          auto p1e = std::make_shared<EdgeLine2DPointXY>();
          p1e->vertices()[0] = line;
          p1e->vertices()[1] = p1;
          p1e->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> p1i;
          p1i(0, 0) = 1e6;
          p1e->setInformation(p1i);
          outGraph.addEdge(p1e);
        }
        if (!p2) {
          p2 = std::make_shared<VertexPointXY>();
          p2->setEstimate(segment->estimateP2());
          p2->setId(currentId++);
          outGraph.addVertex(p2);
          line->p2Id = p2->id();

          auto p2e = std::make_shared<EdgeLine2DPointXY>();
          p2e->vertices()[0] = line;
          p2e->vertices()[1] = p2;
          p2e->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> p2i;
          p2i(0, 0) = 1e6;
          p2e->setInformation(p2i);
          outGraph.addEdge(p2e);
        }

        auto p1e = std::make_shared<EdgeSE2PointXY>();
        p1e->vertices()[0] = pose;
        p1e->vertices()[1] = p1;
        p1e->setMeasurement(es->measurementP1());
        Matrix2 p1i = si.block<2, 2>(0, 0);
        p1e->setInformation(p1i);
        outGraph.addEdge(p1e);

        auto p2e = std::make_shared<EdgeSE2PointXY>();
        p2e->vertices()[0] = pose;
        p2e->vertices()[1] = p2;
        p2e->setMeasurement(es->measurementP2());
        Matrix2 p2i = si.block<2, 2>(2, 2);
        p2e->setInformation(p2i);
        outGraph.addEdge(p2e);
      }

      if (espl) {
        Matrix3 si = espl->information();
        Vector2 lparams;
        lparams[0] = espl->theta();
        Vector2 n(cos(espl->theta()), sin(espl->theta()));
        lparams[1] = n.dot(espl->point());
        Matrix2 li;
        li << si(2, 2), 0, 0, 1000;
        el2->setMeasurement(Line2D(lparams));
        el2->setInformation(li);
        outGraph.addEdge(el2);

        auto& pX = (espl->pointNum() == 0) ? p1 : p2;
        if (!pX) {
          cerr << "mkp: " << line->id() << endl;
          pX = std::make_shared<VertexPointXY>();
          pX->setId(currentId++);
          outGraph.addVertex(pX);

          Vector2 estPx;
          if (espl->pointNum()) {
            estPx = segment->estimateP1();
            line->p1Id = pX->id();
          } else {
            estPx = segment->estimateP2();
            line->p2Id = pX->id();
          }
          pX->setEstimate(estPx);

          auto pXe = std::make_shared<EdgeLine2DPointXY>();
          pXe->vertices()[0] = line;
          pXe->vertices()[1] = pX;
          pXe->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> pXi;
          pXi(0, 0) = 1e6;
          pXe->setInformation(pXi);
          outGraph.addEdge(pXe);
        }

        auto pXe = std::make_shared<EdgeSE2PointXY>();
        pXe->vertices()[0] = pose;
        pXe->vertices()[1] = pX;
        pXe->setMeasurement(espl->point());
        Matrix2 pXi = si.block<2, 2>(0, 0);
        pXe->setInformation(pXi);
        outGraph.addEdge(pXe);
      }
    }
  }

  if (!outputfilename.empty()) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      outGraph.save(std::cout);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      outGraph.save(outputfilename.c_str());
    }
    cerr << "done." << endl;
  }

  // destroy all the singletons
  // Factory::destroy();
  // OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();

  return 0;
}

}  // namespace g2o

int main(int argc, char** argv) { return g2o::run_main(argc, argv); }
