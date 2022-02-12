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

#ifndef G2O_BASE_VERTEX_H
#define G2O_BASE_VERTEX_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stack>

#include "creators.h"
#include "g2o/stuff/macros.h"
#include "optimizable_graph.h"

namespace g2o {
#define G2O_VERTEX_DIM ((D == Eigen::Dynamic) ? dimension_ : D)
/**
 * \brief Templatized BaseVertex
 *
 * Templatized BaseVertex
 * D  : minimal dimension of the vertex, e.g., 3 for rotation in 3D. -1 means
 * dynamically assigned at runtime. T  : internal type to represent the
 * estimate, e.g., Quaternion for rotation in 3D
 */
template <int D, typename T>
class BaseVertex : public OptimizableGraph::Vertex {
 public:
  using EstimateType = T;
  using BackupStackType = std::stack<
      EstimateType,
      std::vector<EstimateType, Eigen::aligned_allocator<EstimateType>>>;

  static const int kDimension =
      D;  ///< dimension of the estimate (minimal) in the manifold space

  using HessianBlockType =
      Eigen::Map<Eigen::Matrix<number_t, D, D, Eigen::ColMajor>,
                 Eigen::Matrix<number_t, D, D, Eigen::ColMajor>::Flags &
                         Eigen::PacketAccessBit
                     ? Eigen::Aligned
                     : Eigen::Unaligned>;

  BaseVertex();

  const number_t& hessian(int i, int j) const override {
    assert(i < G2O_VERTEX_DIM && j < G2O_VERTEX_DIM);
    return hessian_(i, j);
  }
  number_t& hessian(int i, int j) override {
    assert(i < G2O_VERTEX_DIM && j < G2O_VERTEX_DIM);
    return hessian_(i, j);
  }
  number_t hessianDeterminant() const override {
    return hessian_.determinant();
  }
  number_t* hessianData() override {
    return const_cast<number_t*>(hessian_.data());
  }

  inline void mapHessianMemory(number_t* d) override;

  int copyB(number_t* b) const override {
    const int vertexDim = G2O_VERTEX_DIM;
    memcpy(b, b_.data(), vertexDim * sizeof(number_t));
    return vertexDim;
  }

  const number_t& b(int i) const override {
    assert(i < D);
    return b_(i);
  }
  number_t& b(int i) override {
    assert(i < G2O_VERTEX_DIM);
    return b_(i);
  }
  number_t* bData() override { return b_.data(); }

  inline void clearQuadraticForm() override;

  //! updates the current vertex with the direct solution x += H_ii\b_ii
  //! @returns the determinant of the inverted hessian
  inline number_t solveDirect(number_t lambda = 0) override;

  //! return right hand side b of the constructed linear system
  Eigen::Matrix<number_t, D, 1, Eigen::ColMajor>& b() { return b_; }
  const Eigen::Matrix<number_t, D, 1, Eigen::ColMajor>& b() const { return b_; }
  //! return the hessian block associated with the vertex
  HessianBlockType& A() { return hessian_; }
  const HessianBlockType& A() const { return hessian_; }

  void push() override { backup_.push(estimate_); }
  void pop() override {
    assert(!backup_.empty());
    estimate_ = backup_.top();
    backup_.pop();
    updateCache();
  }
  void discardTop() override {
    assert(!backup_.empty());
    backup_.pop();
  }
  int stackSize() const override { return backup_.size(); }

  //! return the current estimate of the vertex
  const EstimateType& estimate() const { return estimate_; }
  //! set the estimate for the vertex also calls updateCache()
  void setEstimate(const EstimateType& et) {
    estimate_ = et;
    updateCache();
  }

 protected:
  HessianBlockType hessian_;
  Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> b_;
  EstimateType estimate_;
  BackupStackType backup_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_vertex.hpp"

#undef G2O_VERTEX_DIM

}  // end namespace g2o

#endif
