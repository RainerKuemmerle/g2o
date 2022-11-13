// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// Copyright (C) 2012 R. Kuemmerle
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

#ifndef G2O_STRUCTURE_ONLY_SOLVER_H
#define G2O_STRUCTURE_ONLY_SOLVER_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer.h"

namespace g2o {

/**
 * \brief This is a solver for "structure-only" optimization"
 *
 * Given the problem of landmark-based SLAM or bundle adjustment, this class
 * performs optimization on the landmarks while the poses are kept fixed. This
 * can be done very efficiently, since the position on the landmarks are
 * independent given the poses are known.
 *
 * This class slightly misuses the API of g2o. It is designed in a way, it can
 * work on the very same graph which reflects the problem of landmark-based
 * SLAM, bundle adjustment and which is meant to be solved using the Schur
 * complement. Thus, it can be called just before or after joint-optimization
 * without the need of additional setup. Call calc() with the point features you
 * want to optimize.
 *
 * This class is still considered as being experimentally!
 */
template <int PointDoF>
class StructureOnlySolver : public OptimizationAlgorithm {
 public:
  StructureOnlySolver() = default;

  OptimizationAlgorithm::SolverResult solve(int iteration,
                                            bool online = false) override {
    (void)iteration;
    (void)online;
    return calc(points_, 1);
  }

  OptimizationAlgorithm::SolverResult calc(
      OptimizableGraph::VertexContainer& vertices, int num_iters,
      int num_max_trials = 10) {
    JacobianWorkspace auxWorkspace;
    auxWorkspace.updateSize(2, 50);
    auxWorkspace.allocate();

    for (auto& v : vertices) {
      bool stop = false;
      assert(v->dimension() == PointDoF);
      const g2o::HyperGraph::EdgeSetWeak& track = v->edges();
      assert(track.size() >= 2);
      number_t chi2 = 0;
      // TODO(Rainer): make these parameters
      number_t mu = cst(0.01);
      number_t nu = 2;

      for (const auto& it_t : track) {
        auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it_t.lock());
        e->computeError();
        if (e->robustKernel()) {
          Vector3 rho;
          e->robustKernel()->robustify(e->chi2(), rho);
          chi2 += rho[0];
        } else {
          chi2 += e->chi2();
        }
      }

      if (!v->fixed()) {
        Eigen::Matrix<number_t, PointDoF, PointDoF, Eigen::ColMajor> H_pp;
        H_pp.resize(v->dimension(), v->dimension());
        v->mapHessianMemory(H_pp.data());
        for (int i_g = 0; i_g < num_iters; ++i_g) {
          H_pp.setZero();
          v->clearQuadraticForm();

          const g2o::HyperGraph::EdgeSetWeak& track = v->edges();
          assert(!track.empty());

          for (const auto& it_t : track) {
            auto e =
                std::static_pointer_cast<OptimizableGraph::Edge>(it_t.lock());

            // fix all the other vertices and remember their fix value
#ifdef WINDOWS
            std::vector<bool> remember_fix_status(e->vertices().size());
#else
            bool remember_fix_status[e->vertices().size()];
#endif
            for (size_t k = 0; k < e->vertices().size(); ++k) {
              auto otherV = std::static_pointer_cast<OptimizableGraph::Vertex>(
                  e->vertex(k));
              if (otherV != v) {
                remember_fix_status[k] = otherV->fixed();
                otherV->setFixed(true);
              }
            }

            // build the matrix
            e->computeError();
            e->linearizeOplus(auxWorkspace);
            e->constructQuadraticForm();

            // Restore frame's initial fixed() values
            for (size_t k = 0; k < e->vertices().size(); ++k) {
              auto otherV = std::static_pointer_cast<OptimizableGraph::Vertex>(
                  e->vertex(k));
              if (otherV != v) {
                otherV->setFixed(remember_fix_status[k]);
              }
            }
          }

          using PointVector =
              Eigen::Matrix<number_t, PointDoF, 1, Eigen::ColMajor>;
          Eigen::Map<PointVector> b(v->bData(), v->dimension());
          if (b.norm() < 0.001) {
            stop = true;
            break;
          }

          int trial = 0;
          do {
            using PointMatrix =
                Eigen::Matrix<number_t, PointDoF, PointDoF, Eigen::ColMajor>;
            PointMatrix H_pp_mu = H_pp;
            H_pp_mu.diagonal().array() += mu;
            Eigen::LDLT<PointMatrix> chol_H_pp(H_pp_mu);
            bool goodStep = false;
            if (chol_H_pp.isPositive()) {
              PointVector delta_p = chol_H_pp.solve(b);
              v->push();
              v->oplus(VectorX::MapType(delta_p.data(), delta_p.size()));
              number_t new_chi2 = 0;
              for (const auto& it_t : track) {
                auto e = std::static_pointer_cast<OptimizableGraph::Edge>(
                    it_t.lock());
                e->computeError();
                if (e->robustKernel()) {
                  Vector3 rho;
                  e->robustKernel()->robustify(e->chi2(), rho);
                  new_chi2 += rho[0];
                } else {
                  new_chi2 += e->chi2();
                }
              }
              assert(g2o_isnan(new_chi2) == false && "Chi is NaN");
              const number_t rho = (chi2 - new_chi2);
              if (rho > 0 && g2o_isfinite(new_chi2)) {
                goodStep = true;
                chi2 = new_chi2;
                v->discardTop();
              } else {
                goodStep = false;
                v->pop();
              }
            }

            // update the damping factor based on the result of the last
            // increment
            if (goodStep) {
              mu *= cst(1. / 3.);
              nu = 2.;
              trial = 0;
              break;  // TODO(Rainer): Revisit the rule to break here
            }
            mu *= nu;
            nu *= 2.;
            ++trial;
            if (trial >= num_max_trials) {
              stop = true;
              break;
            }
          } while (!stop);
          if (stop) break;
        }
      }
    }
    return kOk;
  }

  bool init(bool) override {
    // collect the vertices
    points_.clear();
    for (const auto& v : optimizer()->activeVertices()) {
      if (v->marginalized()) points_.push_back(v);
    }
    return true;
  }

  bool computeMarginals(SparseBlockMatrix<MatrixX>&,
                        const std::vector<std::pair<int, int> >&) override {
    return false;
  }

  bool updateStructure(const HyperGraph::VertexContainer& /*vset*/,
                       const HyperGraph::EdgeSet& /*edges*/) override {
    return true;
  }

  //! return the points of the optimization problem
  OptimizableGraph::VertexContainer& points() { return points_; }
  const OptimizableGraph::VertexContainer& points() const { return points_; }

 protected:
  bool verbose_ = true;
  OptimizableGraph::VertexContainer points_;
};

}  // namespace g2o

#endif
