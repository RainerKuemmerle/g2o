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

#ifndef G2O_BASE_VARIABLE_SIZED_EDGE_H
#define G2O_BASE_VARIABLE_SIZED_EDGE_H

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <vector>

#include "base_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/misc.h"
#include "robust_kernel.h"  // IWYU pragma: keep

namespace g2o {

/**
 * \brief base class to represent an edge connecting an arbitrary number of
 * nodes
 *
 * D - Dimension of the measurement
 * E - type to represent the measurement
 */
template <int D, typename E>
class BaseVariableSizedEdge : public BaseEdge<D, E> {
 public:
  /**
   * \brief helper for mapping the Hessian memory of the upper triangular block
   */
  struct HessianHelper {
    Eigen::Map<MatrixX> matrix;  ///< the mapped memory
    bool transposed = false;     ///< the block has to be transposed
    HessianHelper() : matrix(nullptr, 0, 0) {}
  };

  static constexpr int kDimension = BaseEdge<D, E>::kDimension;
  using Measurement = typename BaseEdge<D, E>::Measurement;
  using JacobianType = MatrixX::MapType;
  using ErrorVector = typename BaseEdge<D, E>::ErrorVector;
  using InformationType = typename BaseEdge<D, E>::InformationType;
  using HessianBlockType =
      Eigen::Map<MatrixX, MatrixX::Flags & Eigen::PacketAccessBit
                              ? Eigen::Aligned
                              : Eigen::Unaligned>;

  BaseVariableSizedEdge() : BaseEdge<D, E>() {}

  void linearizeOplus(JacobianWorkspace& jacobianWorkspace) override;

  /**
   * Linearizes the oplus operator in the vertex, and stores
   * the result in temporary variable vector _jacobianOplus
   */
  virtual void linearizeOplus();

  void resize(size_t size) override;

  [[nodiscard]] bool allVerticesFixed() const override;

  void constructQuadraticForm() override;

  void mapHessianMemory(double* d, int i, int j, bool rowMajor) override;

  using BaseEdge<D, E>::computeError;

 protected:
  using BaseEdge<D, E>::measurement_;
  using BaseEdge<D, E>::information_;
  using BaseEdge<D, E>::error_;
  using BaseEdge<D, E>::vertices_;
  using BaseEdge<D, E>::dimension_;

  std::vector<HessianHelper> hessian_;
  std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> >
      jacobianOplus_;  ///< jacobians of the edge (w.r.t. oplus)

  void computeQuadraticForm(const InformationType& omega,
                            const ErrorVector& weightedError);

  [[nodiscard]] OptimizableGraph::Vertex* vertexRaw(size_t n) const {
    assert(n < vertices_.size() && "Index out of bounds");
    return static_cast<OptimizableGraph::Vertex*>(vertices_[n].get());
  }

 public:
};

namespace internal {
inline int computeUpperTriangleIndex(int i, int j) {
  int elemsUpToCol = ((j - 1) * j) / 2;
  return elemsUpToCol + i;
}
}  // namespace internal

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::constructQuadraticForm() {
  if (this->robustKernel()) {
    double error = this->chi2();
    Vector3 rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<double, D, 1, Eigen::ColMajor> omega_r =
        -information_ * error_;
    omega_r *= rho[1];
    computeQuadraticForm(this->robustInformation(rho), omega_r);
  } else {
    computeQuadraticForm(information_, -information_ * error_);
  }
}

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::linearizeOplus(
    JacobianWorkspace& jacobianWorkspace) {
  for (size_t i = 0; i < vertices_.size(); ++i) {
    OptimizableGraph::Vertex* v = vertexRaw(i);
    assert(v->dimension() >= 0);
    new (&jacobianOplus_[i])
        JacobianType(jacobianWorkspace.workspaceForVertex(i),
                     D < 0 ? dimension_ : D, v->dimension());
  }
  linearizeOplus();
}

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::linearizeOplus() {
  constexpr double kDelta = cst(1e-9);
  constexpr double kScalar = 1 / (2 * kDelta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = error_;

  for (size_t i = 0; i < vertices_.size(); ++i) {
    // Xi - estimate the jacobian numerically
    OptimizableGraph::Vertex* vi = vertexRaw(i);

    if (vi->fixed()) {
      continue;
    }

    internal::QuadraticFormLock lck(*vi);
    const int vi_dim = vi->dimension();
    assert(vi_dim >= 0);

    assert(dimension_ >= 0);
    assert(jacobianOplus_[i].rows() == dimension_ &&
           jacobianOplus_[i].cols() == vi_dim &&
           "jacobian cache dimension does not match");
    jacobianOplus_[i].resize(dimension_, vi_dim);
    // add small step along the unit vector in each dimension
    VectorX add_vi_buffer(vi_dim);
    add_vi_buffer.fill(0.);
    VectorX::MapType add_vi(add_vi_buffer.data(), add_vi_buffer.size());
    for (int d = 0; d < vi_dim; ++d) {
      vi->push();
      add_vi[d] = kDelta;
      vi->oplus(add_vi);
      computeError();
      errorBak = error_;
      vi->pop();
      vi->push();
      add_vi[d] = -kDelta;
      vi->oplus(add_vi);
      computeError();
      errorBak -= error_;
      vi->pop();
      add_vi[d] = 0.0;

      jacobianOplus_[i].col(d) = kScalar * errorBak;
    }  // end dimension
    error_ = errorBeforeNumeric;
  }
}

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::mapHessianMemory(double* d, int i, int j,
                                                   bool rowMajor) {
  int idx = internal::computeUpperTriangleIndex(i, j);
  assert(idx < (int)hessian_.size());
  OptimizableGraph::Vertex* vi = vertexRaw(i);
  OptimizableGraph::Vertex* vj = vertexRaw(j);
  assert(vi->dimension() >= 0);
  assert(vj->dimension() >= 0);
  HessianHelper& h = hessian_[idx];
  if (rowMajor) {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vj->dimension(), vi->dimension());
  } else {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vi->dimension(), vj->dimension());
  }
  h.transposed = rowMajor;
}

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::resize(size_t size) {
  BaseEdge<D, E>::resize(size);
  int n = static_cast<int>(vertices_.size());
  int maxIdx = (n * (n - 1)) / 2;
  assert(maxIdx >= 0);
  hessian_.resize(maxIdx);
  jacobianOplus_.resize(size, JacobianType(nullptr, 0, 0));
}

template <int D, typename E>
bool BaseVariableSizedEdge<D, E>::allVerticesFixed() const {
  for (size_t i = 0; i < vertices_.size(); ++i) {
    if (!vertexRaw(i)->fixed()) {
      return false;
    }
  }
  return true;
}

template <int D, typename E>
void BaseVariableSizedEdge<D, E>::computeQuadraticForm(
    const InformationType& omega, const ErrorVector& weightedError) {
  for (size_t i = 0; i < vertices_.size(); ++i) {
    OptimizableGraph::Vertex* from = vertexRaw(i);
    bool istatus = !(from->fixed());

    if (istatus) {
      const JacobianType& A = jacobianOplus_[i];

      MatrixX AtO = A.transpose() * omega;
      int fromDim = from->dimension();
      assert(fromDim >= 0);
      Eigen::Map<MatrixX> fromMap(from->hessianData(), fromDim, fromDim);
      Eigen::Map<VectorX> fromB(from->bData(), fromDim);

      // ii block in the hessian
      {
        internal::QuadraticFormLock lck(*from);
        fromMap.noalias() += AtO * A;
        fromB.noalias() += A.transpose() * weightedError;
      }

      // compute the off-diagonal blocks ij for all j
      for (size_t j = i + 1; j < vertices_.size(); ++j) {
        OptimizableGraph::Vertex* to = vertexRaw(j);

        bool jstatus = !(to->fixed());
        if (jstatus) {
          internal::QuadraticFormLock lck(*to);
          const JacobianType& B = jacobianOplus_[j];
          int idx = internal::computeUpperTriangleIndex(i, j);
          assert(idx < (int)hessian_.size());
          HessianHelper& hhelper = hessian_[idx];
          if (hhelper
                  .transposed) {  // we have to write to the block as transposed
            hhelper.matrix.noalias() += B.transpose() * AtO.transpose();
          } else {
            hhelper.matrix.noalias() += AtO * B;
          }
        }
      }
    }
  }
}

}  // end namespace g2o

#endif
