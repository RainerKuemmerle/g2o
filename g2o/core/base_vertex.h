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
#include <cassert>
#include <stack>

#include "creators.h"
#include "g2o/config.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/macros.h"
#include "optimizable_graph.h"

namespace g2o {

#define G2O_VERTEX_DIM ((D == Eigen::Dynamic) ? dimension_ : D)

/**
 * \brief Templatized BaseVertex
 *
 * Templatized BaseVertex
 *
 * D: minimal dimension of the vertex, e.g., 3 for rotation in 3D. -1 means
 * dynamically assigned at runtime.
 * T: internal type to represent the estimate, e.g., Quaternion for rotation in
 * 3D
 */
template <int D, typename T>
class BaseVertex : public OptimizableGraph::Vertex {
 public:
  using EstimateType = T;
  using BackupStackType = std::stack<EstimateType, std::vector<EstimateType> >;

  static const int kDimension =
      D;  ///< dimension of the estimate (minimal) in the manifold space

  using BVector = VectorN<D>;
  using HessianBlockType =
      Eigen::Map<MatrixN<D>, MatrixN<D>::Flags & Eigen::PacketAccessBit
                                 ? Eigen::Aligned
                                 : Eigen::Unaligned>;

  BaseVertex();
  BaseVertex& operator=(const BaseVertex&) = delete;
  BaseVertex(const BaseVertex&) = delete;

  [[nodiscard]] double* hessianData() const final {
    return const_cast<double*>(hessian_.data());
  }

  void mapHessianMemory(double* d) final;

  int copyB(double* b) const final {
    const int vertexDim = G2O_VERTEX_DIM;
    memcpy(b, b_.data(), vertexDim * sizeof(double));
    return vertexDim;
  }

  [[nodiscard]] double* bData() const final {
    return const_cast<double*>(b_.data());
  }

  void clearQuadraticForm() final { b_.setZero(); }

  //! updates the current vertex with the direct solution x += H_ii\b_ii
  //! @returns the determinant of the inverted hessian
  double solveDirect(double lambda = 0) override;

  //! return right hand side b of the constructed linear system
  BVector& b() { return b_; }
  const BVector& b() const { return b_; }
  //! return the hessian block associated with the vertex
  HessianBlockType& A() { return hessian_; }
  const HessianBlockType& A() const { return hessian_; }

  void push() final { backup_.push(estimate_); }
  void pop() final {
    assert(!backup_.empty());
    estimate_ = backup_.top();
    backup_.pop();
    updateCache();
  }
  void discardTop() final {
    assert(!backup_.empty());
    backup_.pop();
  }
  [[nodiscard]] int stackSize() const final { return backup_.size(); }

  //! return the current estimate of the vertex
  const EstimateType& estimate() const { return estimate_; }
  //! set the estimate for the vertex also calls updateCache()
  void setEstimate(const EstimateType& et) {
    estimate_ = et;
    updateCache();
  }

  //! generic read based on Trait reading a vector
  bool read(std::istream& is) override {
    typename TypeTraits<EstimateType>::VectorType estimate_data;
    bool state = internal::readVector(is, estimate_data);
    setEstimate(TypeTraits<EstimateType>::fromVector(estimate_data));
    return state;
  }

  //! generic write  based on Trait reading a vector
  bool write(std::ostream& os) const override {
    return internal::writeVector(
        os, TypeTraits<EstimateType>::toVector(estimate()));
  }

  // methods based on the traits interface
  void setToOriginImpl() final {
    setEstimate(TypeTraits<EstimateType>::Identity());
  }

  bool setEstimateData(const double* est) final {
    static_assert(TypeTraits<EstimateType>::kVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    typename TypeTraits<EstimateType>::VectorType::ConstMapType aux(
        est, DimensionTraits<EstimateType>::dimension(estimate_));
    estimate_ = TypeTraits<EstimateType>::fromVector(aux);
    updateCache();
    return true;
  }

  bool getEstimateData(double* est) const final {
    static_assert(TypeTraits<EstimateType>::kVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    TypeTraits<EstimateType>::toData(estimate(), est);
    return true;
  }

  [[nodiscard]] int estimateDimension() const final {
    static_assert(TypeTraits<EstimateType>::kVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    return TypeTraits<EstimateType>::kVectorDimension;
  }

  bool setMinimalEstimateData(const double* est) final {
    static_assert(TypeTraits<EstimateType>::kMinimalVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    typename TypeTraits<EstimateType>::MinimalVectorType::ConstMapType aux(
        est, DimensionTraits<EstimateType>::minimalDimension(estimate()));
    setEstimate(TypeTraits<EstimateType>::fromMinimalVector(aux));
    return true;
  }

  bool getMinimalEstimateData(double* est) const final {
    static_assert(TypeTraits<EstimateType>::kMinimalVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    TypeTraits<EstimateType>::toMinimalData(estimate(), est);
    return true;
  }

  [[nodiscard]] int minimalEstimateDimension() const final {
    static_assert(TypeTraits<EstimateType>::kMinimalVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    return TypeTraits<EstimateType>::kMinimalVectorDimension;
  }

 protected:
  HessianBlockType hessian_;
  BVector b_;
  EstimateType estimate_;
  BackupStackType backup_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_vertex.hpp"

#undef G2O_VERTEX_DIM

}  // end namespace g2o

#endif
