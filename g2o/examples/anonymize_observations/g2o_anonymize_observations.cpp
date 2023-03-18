// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"

namespace g2o {

template <typename T>
bool anonymizeLandmarkEdge(const std::shared_ptr<HyperGraph::Edge>& e_,
                           OptimizableGraph& g) {
  auto e = std::dynamic_pointer_cast<T>(e_);
  if (!e) return false;
  g.setEdgeVertex(e, 1, nullptr);
  return true;
}

template <typename T>
bool anonymizePoseEdge(const std::shared_ptr<HyperGraph::Edge>& e_,
                       OptimizableGraph& g) {
  auto e = std::dynamic_pointer_cast<T>(e_);
  if (!e) return false;
  auto from = e->vertex(0);
  auto to = e->vertex(1);
  if (from && to && from != to) {
    int deltaId = abs(from->id() - to->id());
    if (deltaId > 1) {
      if (from->id() > to->id()) {
        g.setEdgeVertex(e, 0, nullptr);
      } else {
        g.setEdgeVertex(e, 1, nullptr);
      }
      return true;
    }
  }
  return false;
}

}  // namespace g2o

int main(int argc, char** argv) {
  g2o::CommandArgs arg;
  std::string outputFilename;
  std::string inputFilename;
  arg.param("o", outputFilename, "anon.g2o", "output file");
  arg.paramLeftOver("graph-output", inputFilename, "",
                    "graph file which will be read", true);
  arg.parseArgs(argc, argv);
  g2o::OptimizableGraph graph;

  if (inputFilename.empty()) {
    std::cerr << "No input data specified" << std::endl;
    return 0;
  }
  if (inputFilename == "-") {
    std::cerr << "Read input from stdin" << std::endl;
    if (!graph.load(std::cin)) {
      std::cerr << "Error loading graph" << std::endl;
      return 2;
    }
  } else {
    std::cerr << "Read input from " << inputFilename << std::endl;
    std::ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      std::cerr << "Failed to open file" << std::endl;
      return 1;
    }
    if (!graph.load(ifs)) {
      std::cerr << "Error loading graph" << std::endl;
      return 2;
    }
  }

  for (auto it = graph.edges().begin(); it != graph.edges().end(); ++it) {
    const auto& e = *it;
    // clang-format off
    if (g2o::anonymizeLandmarkEdge<g2o::EdgeSE2PointXY>(e, graph)) continue;
    if (g2o::anonymizeLandmarkEdge<g2o::EdgeSE2PointXYOffset>(e, graph)) continue;
    if (g2o::anonymizeLandmarkEdge<g2o::EdgeSE2PointXYBearing>(e, graph)) continue;
    if (g2o::anonymizePoseEdge<g2o::EdgeSE2>(e, graph)) continue;
    if (g2o::anonymizePoseEdge<g2o::EdgeSE2Offset>(e, graph)) continue;
    // clang-format on
  }

  std::ofstream os(outputFilename.c_str());
  graph.save(os);
}
