// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "simple_star_ops.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <iostream>

#include "backbone_tree_action.h"
#include "edge_types_cost_function.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"

using std::cerr;
using std::endl;

namespace g2o {

double activeVertexChi(const OptimizableGraph::Vertex* v) {
  const auto* s = dynamic_cast<const SparseOptimizer*>(v->graph());
  const OptimizableGraph::EdgeContainer& av = s->activeEdges();
  double chi = 0;
  int ne = 0;
  for (const auto& it : v->edges()) {
    auto e = std::dynamic_pointer_cast<OptimizableGraph::Edge>(it.lock());
    if (!e) continue;
    if (s->findActiveEdge(e.get()) != av.end()) {
      chi += e->chi2();
      ne++;
    }
  }
  if (!ne) return -1;
  return chi / ne;
}

void constructEdgeStarMap(EdgeStarMap& esmap, StarSet& stars, bool low) {
  esmap.clear();
  for (const auto& s : stars) {
    if (low) {
      for (auto it = s->lowLevelEdges().begin(); it != s->lowLevelEdges().end();
           ++it) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(*it);
        esmap.insert(make_pair(e, s));
      }
    } else {
      for (auto it = s->starEdges().begin(); it != s->starEdges().end(); ++it) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(*it);
        esmap.insert(make_pair(e, s));
      }
    }
  }
}

size_t vertexEdgesInStar(HyperGraph::EdgeSet& eset,
                         const std::shared_ptr<OptimizableGraph::Vertex>& v,
                         const std::shared_ptr<Star>& s, EdgeStarMap& esmap) {
  eset.clear();
  for (const auto& it : v->edges()) {
    auto e = std::dynamic_pointer_cast<OptimizableGraph::Edge>(it.lock());
    auto eit = esmap.find(e);
    if (eit != esmap.end() && eit->second == s) eset.insert(e);
  }
  return eset.size();
}

void starsInVertex(StarSet& stars,
                   const std::shared_ptr<OptimizableGraph::Vertex>& v,
                   EdgeStarMap& esmap) {
  for (const auto& it : v->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it.lock());
    auto eit = esmap.find(e);
    if (eit != esmap.end()) stars.insert(eit->second);
  }
}

void starsInEdge(StarSet& stars,
                 const std::shared_ptr<OptimizableGraph::Edge>& e,
                 EdgeStarMap& esmap, HyperGraph::VertexSet& gauge) {
  for (auto& i : e->vertices()) {
    auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(i);
    if (gauge.find(v) == gauge.end()) starsInVertex(stars, v, esmap);
  }
}

void assignHierarchicalEdges(StarSet& stars, EdgeStarMap& esmap,
                             EdgeLabeler* labeler, EdgeCreator* creator,
                             SparseOptimizer* optimizer, int minNumEdges,
                             int maxIterations) {
  // now construct the hierarchical edges for all the stars
  for (const auto& s : stars) {
    std::vector<std::shared_ptr<OptimizableGraph::Vertex>> vertices(2);
    vertices[0] =
        std::static_pointer_cast<OptimizableGraph::Vertex>(*s->gauge().begin());
    HyperGraph::VertexSet vNew = s->lowLevelVertices();
    for (auto vit = s->lowLevelVertices().begin();
         vit != s->lowLevelVertices().end(); ++vit) {
      auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(*vit);
      vertices[1] = v;
      if (v == vertices[0]) continue;
      HyperGraph::EdgeSet eInSt;
      const int numEdges = vertexEdgesInStar(eInSt, v, s, esmap);
      if (Factory::instance()->tag(v.get()) ==
              Factory::instance()->tag(vertices[0].get()) ||
          numEdges > minNumEdges) {
        const std::shared_ptr<OptimizableGraph::Edge> e =
            creator->createEdge(vertices);
        // cerr << "creating edge" << e << endl;
        if (e) {
          e->setLevel(1);
          optimizer->addEdge(e);
          s->starEdges().insert(e);
        }
      } else {
        vNew.erase(v);
        // remove from the star all edges that are not sufficiently connected
        for (const auto& it : eInSt) {
          s->lowLevelEdges().erase(it);
        }
      }
    }
    s->lowLevelVertices() = vNew;

    const bool labelOk = s->labelStarEdges(maxIterations, labeler);
    (void)labelOk;
  }
}

void computeBorder(StarSet& stars, EdgeStarMap& hesmap) {
  for (const auto& s : stars) {
    for (auto iit = s->starEdges().begin(); iit != s->starEdges().end();
         ++iit) {
      auto e = std::static_pointer_cast<OptimizableGraph::Edge>(*iit);
      StarSet sset;
      starsInEdge(sset, e, hesmap, s->gauge());
      // cerr << "e: " << e << " l:" << e->level() << " sset.size()=" <<
      // sset.size() << endl;
      if (sset.size() > 1) {
        s->starFrontierEdges().insert(e);
      }
    }
  }
}

void computeSimpleStars(StarSet& stars, SparseOptimizer* optimizer,
                        EdgeLabeler* labeler, EdgeCreator* creator,
                        const std::shared_ptr<OptimizableGraph::Vertex>& gauge,
                        const std::string& edgeTag,
                        const std::string& vertexTag, int level, int step,
                        int backboneIterations, int starIterations,
                        double rejectionThreshold, bool debug) {
  auto pointerWrapper =
      std::shared_ptr<HyperGraph>(optimizer, [](HyperGraph*) {});
  HyperDijkstra d(pointerWrapper);
  // compute a spanning tree based on the types of edges and vertices in the
  // pool
  EdgeTypesCostFunction f(edgeTag, vertexTag, level);
  d.shortestPaths(gauge, f, std::numeric_limits<double>::max(), 1e-6, false,
                  std::numeric_limits<double>::max() / 2);

  HyperDijkstra::computeTree(d.adjacencyMap());
  // constructs the stars on the backbone

  BackBoneTreeAction bact(optimizer, vertexTag, level, step);
  bact.init();

  // perform breadth-first visit of the visit tree and create the stars on the
  // backbone
  g2o::HyperDijkstra::visitAdjacencyMap(d.adjacencyMap(), bact, true);
  stars.clear();

  for (auto& it : bact.vertexStarMultiMap()) {
    stars.insert(it.second);
  }

  //  for each star

  //    for all vertices in the backbone, select all edges leading/leaving from
  //    that vertex that are contained in freeEdges.

  //      mark the corresponding "open" vertices and add them to a multimap
  //      (vertex->star)

  //    select a gauge in the backbone

  //    push all vertices on the backbone

  //    compute an initial guess on the backbone

  //    one round of optimization backbone

  //    lock all vertices in the backbone

  //    push all "open" vertices

  //    for each open vertex,
  //      compute an initial guess given the backbone
  //      do some rounds of solveDirect
  //      if (fail)
  //        - remove the vertex and the edges in that vertex from the star
  //   - make the structures consistent

  //    pop all "open" vertices
  //    pop all "vertices" in the backbone
  //    unfix the vertices in the backbone

  int starNum = 0;
  for (const auto& s : stars) {
    HyperGraph::VertexSet& backboneVertices = s->lowLevelVertices();
    HyperGraph::EdgeSet& backboneEdges = s->lowLevelEdges();
    if (backboneEdges.empty()) continue;

    // cerr << "optimizing backbone" << endl;
    // one of these  should be the gauge, to be simple we select the first one
    // in the backbone
    OptimizableGraph::VertexSet gauge;
    gauge.insert(*backboneVertices.begin());
    s->gauge() = gauge;
    s->optimizer()->push(backboneVertices);
    s->optimizer()->setFixed(gauge, true);
    s->optimizer()->initializeOptimization(backboneEdges);
    s->optimizer()->computeInitialGuess();
    s->optimizer()->optimize(backboneIterations);
    s->optimizer()->setFixed(backboneVertices, true);

    // cerr << "assignind edges.vertices not in bbone" << endl;
    HyperGraph::EdgeSet otherEdges;
    HyperGraph::VertexSet otherVertices;
    for (auto bit = backboneVertices.begin(); bit != backboneVertices.end();
         ++bit) {
      HyperGraph::Vertex* v = bit->get();
      for (const auto& eit : v->edges()) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(eit.lock());
        auto feit = bact.freeEdges().find(e);
        if (feit != bact.freeEdges().end()) {  // edge is admissible
          otherEdges.insert(e);
          bact.freeEdges().erase(feit);
          for (auto& i : e->vertices()) {
            auto ve = std::static_pointer_cast<OptimizableGraph::Vertex>(i);
            if (backboneVertices.find(ve) == backboneVertices.end()) {
              otherVertices.insert(ve);
            }
          }
        }
      }
    }

    // RAINER TODO maybe need a better solution than dynamic casting here??
    OptimizationAlgorithmWithHessian* solverWithHessian =
        dynamic_cast<OptimizationAlgorithmWithHessian*>(
            s->optimizer()->solver().get());
    if (solverWithHessian) {
      s->optimizer()->push(otherVertices);
      // cerr << "optimizing vertices out of bbone" << endl;
      // cerr << "push" << endl;
      // cerr << "init" << endl;
      s->optimizer()->initializeOptimization(otherEdges);
      // cerr << "guess" << endl;
      s->optimizer()->computeInitialGuess();
      // cerr << "solver init" << endl;
      s->optimizer()->solver()->init();
      // cerr << "structure" << endl;
      if (!solverWithHessian->buildLinearStructure())
        cerr << "FATAL: failure while building linear structure" << endl;
      // cerr << "errors" << endl;
      s->optimizer()->computeActiveErrors();
      // cerr << "system" << endl;
      solverWithHessian->updateLinearSystem();
      // cerr << "directSolove" << endl;
    } else {
      cerr << "FATAL: hierarchical thing cannot be used with a solver that "
              "does not support the system structure "
              "construction"
           << endl;
    }

    // // then optimize the vertices one at a time to check if a solution is
    // good
    for (const auto& otherVertex : otherVertices) {
      auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(otherVertex);
      v->solveDirect();
      // cerr << " " << d;
      // if  a solution is found, add a vertex and all the edges in
      // othervertices that are pointing to that edge to the star
      s->lowLevelVertices().insert(v);
      for (const auto& eit : v->edges()) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(eit.lock());
        if (otherEdges.find(e) != otherEdges.end())
          s->lowLevelEdges().insert(e);
      }
    }
    // cerr <<  endl;

    // relax the backbone and optimize it all
    // cerr << "relax bbone" << endl;
    s->optimizer()->setFixed(backboneVertices, false);
    // cerr << "fox gauge bbone" << endl;
    s->optimizer()->setFixed(s->gauge(), true);

    // cerr << "opt init" << endl;
    s->optimizer()->initializeOptimization(s->lowLevelEdges());
    optimizer->computeActiveErrors();
    const int starOptResult = s->optimizer()->optimize(starIterations);
    // cerr << starOptResult << "(" << starIterations << ")  " << endl;

    if (!starIterations || starOptResult > 0) {
      optimizer->computeActiveErrors();

#if 1

      s->optimizer()->computeActiveErrors();
      // cerr << "system" << endl;
      if (solverWithHessian) solverWithHessian->updateLinearSystem();
      HyperGraph::EdgeSet prunedStarEdges = backboneEdges;
      HyperGraph::VertexSet prunedStarVertices = backboneVertices;
      for (const auto& otherVertex : otherVertices) {
        // discard the vertices whose error is too big
        auto v =
            std::static_pointer_cast<OptimizableGraph::Vertex>(otherVertex);
        const MatrixX h = v->hessianMap();
        Eigen::EigenSolver<MatrixX> esolver;
        esolver.compute(h);
        Eigen::VectorXcd ev = esolver.eigenvalues();
        double emin = std::numeric_limits<double>::max();
        double emax = -std::numeric_limits<double>::max();
        for (int i = 0; i < ev.size(); i++) {
          emin = ev(i).real() > emin ? emin : ev(i).real();
          emax = ev(i).real() < emax ? emax : ev(i).real();
        }

        const double d = emin / emax;

        // cerr << " " << d;
        if (d > rejectionThreshold) {
          // if  a solution is found, add a vertex and all the edges in
          // othervertices that are pointing to that edge to the star
          prunedStarVertices.insert(v);
          for (const auto& eit : v->edges()) {
            auto e =
                std::static_pointer_cast<OptimizableGraph::Edge>(eit.lock());
            if (otherEdges.find(e) != otherEdges.end())
              prunedStarEdges.insert(e);
          }
          // cerr << "K( " << v->id() << "," << d << ")" ;
        }
      }
      s->lowLevelEdges() = prunedStarEdges;
      s->lowLevelVertices() = prunedStarVertices;

#endif
      // cerr << "addHedges" << endl;
      // now add to the star the hierarchical edges
      OptimizableGraph::VertexContainer vertices(2);
      vertices[0] = std::static_pointer_cast<OptimizableGraph::Vertex>(
          *s->gauge().begin());

      for (auto vit = s->lowLevelVertices().begin();
           vit != s->lowLevelVertices().end(); ++vit) {
        auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(*vit);
        vertices[1] = v;
        if (v == vertices[0]) continue;
        auto e = creator->createEdge(vertices);
        // rr << "creating edge" << e <<  Factory::instance()->tag(vertices[0])
        // << "->" <<  Factory::instance()->tag(v) <endl;
        if (e) {
          e->setLevel(level + 1);
          optimizer->addEdge(e);
          s->starEdges().insert(e);
        }
      }
    }

    if (debug) {
      char starLowName[100];
      sprintf(starLowName, "star-%04d-low.g2o", starNum);
      std::ofstream starLowStream(starLowName);
      optimizer->saveSubset(starLowStream, s->lowLevelEdges());
    }
    bool labelOk = false;
    if (!starIterations || starOptResult > 0)
      labelOk = s->labelStarEdges(0, labeler);
    if (labelOk) {
      if (debug) {
        char starHighName[100];
        sprintf(starHighName, "star-%04d-high.g2o", starNum);
        std::ofstream starHighStream(starHighName);
        optimizer->saveSubset(starHighStream, s->starEdges());
      }
    }
    starNum++;

    // label each hierarchical edge
    s->optimizer()->pop(otherVertices);
    s->optimizer()->pop(backboneVertices);
    s->optimizer()->setFixed(s->gauge(), false);
  }

  // now erase the stars that have 0 edges. They are useless
  StarSet stars2;
  for (const auto& s : stars) {
    if (!s->lowLevelEdges().empty()) stars2.insert(s);
  }
  stars = stars2;
}

}  // namespace g2o
