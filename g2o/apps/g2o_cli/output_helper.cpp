// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "output_helper.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/hyper_graph_action.h"

#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"

#include <Eigen/Core>

#include <iostream>
#include <fstream>
using namespace std;

namespace g2o {

bool edgeAllVertsSameDim(OptimizableGraph::Edge* e, int dim)
{
  for (size_t i = 0; i < e->vertices().size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(e->vertices()[i]);
    if (v->dimension() != dim)
      return false;
  }
  return true;
}

bool saveGnuplot(const std::string& gnudump, const OptimizableGraph& optimizer)
{
  HyperGraph::VertexSet vset;
  for (HyperGraph::VertexIDMap::const_iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); it++){
    vset.insert(it->second);
  }
  return saveGnuplot(gnudump, vset, optimizer.edges());
}

bool saveGnuplot(const std::string& gnudump, const HyperGraph::VertexSet& vertices, const HyperGraph::EdgeSet& edges)
{
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot = HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (! saveGnuplot ){
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered" << endl;
    return false;
  }
  WriteGnuplotAction::Parameters params;

  int maxDim = -1;
  int minDim = numeric_limits<int>::max();
  for (HyperGraph::VertexSet::const_iterator it = vertices.begin(); it != vertices.end(); ++it){
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    int vdim = v->dimension();
    maxDim = (std::max)(vdim, maxDim);
    minDim = (std::min)(vdim, minDim);
  }

  string extension = getFileExtension(gnudump);
  if (extension.size() == 0)
    extension = "dat";
  string baseFilename = getPureFilename(gnudump);

  // check for odometry edges
  bool hasOdomEdge = false;
  bool hasLandmarkEdge = false;
  for (HyperGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    if (e->vertices().size() == 2) {
      if (edgeAllVertsSameDim(e, maxDim))
        hasOdomEdge = true;
      else
        hasLandmarkEdge = true;
    }
    if (hasOdomEdge && hasLandmarkEdge)
      break;
  }

  bool fileStatus = true;
  if (hasOdomEdge) {
    string odomFilename = baseFilename + "_odom_edges." + extension;
    cerr << "# saving " << odomFilename << " ... ";
    ofstream fout(odomFilename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing odometry edges
    for (HyperGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      if (e->vertices().size() != 2 || ! edgeAllVertsSameDim(e, maxDim))
        continue;
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (hasLandmarkEdge) {
    string filename = baseFilename + "_landmarks_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing landmark edges
    for (HyperGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      if (e->vertices().size() != 2 || edgeAllVertsSameDim(e, maxDim))
        continue;
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (1) {
    string filename = baseFilename + "_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing all edges
    for (HyperGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (1) {
    string filename = baseFilename + "_vertices." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    for (HyperGraph::VertexSet::const_iterator it = vertices.begin(); it != vertices.end(); ++it){
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      (*saveGnuplot)(v, &params);
    }
    cerr << "done." << endl;
  }

  return fileStatus;
}

bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer)
{
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot = HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (! saveGnuplot ){
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered" << endl;
    return false;
  }
  WriteGnuplotAction::Parameters params;
  params.os = &os;

  // writing all edges
  os << "set terminal x11 noraise" << endl;
  os << "set size ratio -1" << endl;
  os << "plot \"-\" w l" << endl;
  for (HyperGraph::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    (*saveGnuplot)(e, &params);
  }
  os << "e" << endl;

  return true;
}

} // end namespace
