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

#include "output_helper.h"

#include <Eigen/Core>
#include <fstream>
#include <iostream>

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"

using std::cerr;
using std::endl;
using std::ofstream;
using std::string;

namespace g2o {

static bool edgeAllVertsSameDim(OptimizableGraph::Edge* e, int dim) {
  return std::all_of(e->vertices().cbegin(), e->vertices().cend(),
                     [dim](const std::shared_ptr<HyperGraph::Vertex>& vertex) {
                       auto* v =
                           static_cast<OptimizableGraph::Vertex*>(vertex.get());
                       return v->dimension() == dim;
                     });
}

bool saveGnuplot(const std::string& gnudump,
                 const OptimizableGraph& optimizer) {
  HyperGraph::VertexSet vset;
  for (const auto& it : optimizer.vertices()) {
    vset.insert(it.second);
  }
  return saveGnuplot(gnudump, vset, optimizer.edges());
}

bool saveGnuplot(const std::string& gnudump,
                 const HyperGraph::VertexSet& vertices,
                 const HyperGraph::EdgeSet& edges) {
  // seek for an action whose name is writeGnuplot in the library
  auto saveGnuplot =
      HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (!saveGnuplot) {
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered"
         << endl;
    return false;
  }
  auto params = std::make_shared<WriteGnuplotAction::Parameters>();

  int maxDim = -1;
  int minDim = std::numeric_limits<int>::max();
  for (const auto& vertice : vertices) {
    auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(vertice);
    const int vdim = v->dimension();
    maxDim = (std::max)(vdim, maxDim);
    minDim = (std::min)(vdim, minDim);
  }

  string extension = getFileExtension(gnudump);
  if (extension.empty()) extension = "dat";
  const string baseFilename = getPureFilename(gnudump);

  // check for odometry edges
  bool hasOdomEdge = false;
  bool hasLandmarkEdge = false;
  for (const auto& edge : edges) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
    if (e->vertices().size() == 2) {
      if (edgeAllVertsSameDim(e.get(), maxDim))
        hasOdomEdge = true;
      else
        hasLandmarkEdge = true;
    }
    if (hasOdomEdge && hasLandmarkEdge) break;
  }

  if (hasOdomEdge) {
    string odomFilename = baseFilename + "_odom_edges." + extension;
    cerr << "# saving " << odomFilename << " ... ";
    ofstream fout(odomFilename.c_str());
    if (!fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params->os = &fout;

    // writing odometry edges
    for (const auto& edge : edges) {
      auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
      if (e->vertices().size() != 2 || !edgeAllVertsSameDim(e.get(), maxDim))
        continue;
      (*saveGnuplot)(*e, params);
    }
    cerr << "done." << endl;
  }

  if (hasLandmarkEdge) {
    const string filename = baseFilename + "_landmarks_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (!fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params->os = &fout;

    // writing landmark edges
    for (const auto& edge : edges) {
      auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
      if (e->vertices().size() != 2 || edgeAllVertsSameDim(e.get(), maxDim))
        continue;
      (*saveGnuplot)(*e, params);
    }
    cerr << "done." << endl;
  }

  {
    const string filename = baseFilename + "_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (!fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params->os = &fout;

    // writing all edges
    for (const auto& edge : edges) {
      auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
      (*saveGnuplot)(*e, params);
    }
    cerr << "done." << endl;
  }

  {
    const string filename = baseFilename + "_vertices." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (!fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params->os = &fout;

    for (const auto& vertex : vertices) {
      auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(vertex);
      (*saveGnuplot)(*v, params);
    }
    cerr << "done." << endl;
  }

  // TODO(goki): Determine a proper return value for writing to file
  return true;
}

bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer) {
  // seek for an action whose name is writeGnuplot in the library
  auto saveGnuplot =
      HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (!saveGnuplot) {
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered"
         << endl;
    return false;
  }
  auto params = std::make_shared<WriteGnuplotAction::Parameters>();
  params->os = &os;

  // writing all edges
  os << "set terminal x11 noraise" << endl;
  os << "set size ratio -1" << endl;
  os << "plot \"-\" w l" << endl;
  for (const auto& it : optimizer.edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it);
    (*saveGnuplot)(*e, params);
  }
  os << "e" << endl;

  return true;
}

}  // namespace g2o
