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

#ifndef G2O_BASE_EDGE_H
#define G2O_BASE_EDGE_H

#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <type_traits>

#include "optimizable_graph.h"

namespace g2o {

namespace internal {

#ifdef G2O_OPENMP
struct QuadraticFormLock {
  explicit QuadraticFormLock(OptimizableGraph::Vertex& vertex)
      : _vertex(vertex) {
    _vertex.lockQuadraticForm();
  }
  ~QuadraticFormLock() { _vertex.unlockQuadraticForm(); }

 private:
  OptimizableGraph::Vertex& _vertex;
};
#else
struct QuadraticFormLock {
  explicit QuadraticFormLock(OptimizableGraph::Vertex&) {}
};
#endif

/**
 * Declaring the types for the error vector and the information matrix depending
 * on the size of the error function. In particular, the information matrix
 * needs to match the size of the error vector.
 */
template <int D>
struct BaseEdgeTraits {
  static constexpr int kDimension = D;
  using ErrorVector = Eigen::Matrix<number_t, D, 1, Eigen::ColMajor>;
  using InformationType = Eigen::Matrix<number_t, D, D, Eigen::ColMajor>;
};
/**
 * Same as above but for dimension not known at compilation, i.e., dynamically
 * sized edges.
 */
template <>
struct BaseEdgeTraits<-1> {
  static constexpr int kDimension = -1;
  using ErrorVector =
      Eigen::Matrix<number_t, Eigen::Dynamic, 1, Eigen::ColMajor>;
  using InformationType =
      Eigen::Matrix<number_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
};

}  // namespace internal

template <int D, typename E>
class BaseEdge : public OptimizableGraph::Edge {
 public:
  static constexpr int kDimension = internal::BaseEdgeTraits<D>::kDimension;
  using Measurement = E;
  using ErrorVector = typename internal::BaseEdgeTraits<D>::ErrorVector;
  using InformationType = typename internal::BaseEdgeTraits<D>::InformationType;

  BaseEdge() : OptimizableGraph::Edge() { dimension_ = D; }

  number_t chi2() const override { return error_.dot(information() * error_); }

  const number_t* errorData() const override { return error_.data(); }
  number_t* errorData() override { return error_.data(); }
  const ErrorVector& error() const { return error_; }
  ErrorVector& error() { return error_; }

  //! information matrix of the constraint
  EIGEN_STRONG_INLINE const InformationType& information() const {
    return information_;
  }
  EIGEN_STRONG_INLINE InformationType& information() { return information_; }
  void setInformation(const InformationType& information) {
    information_ = information;
  }

  const number_t* informationData() const override {
    return information_.data();
  }
  number_t* informationData() override { return information_.data(); }

  //! accessor functions for the measurement represented by the edge
  EIGEN_STRONG_INLINE const Measurement& measurement() const {
    return measurement_;
  }
  virtual void setMeasurement(const Measurement& m) { measurement_ = m; }

  virtual int rank() const { return dimension(); }

  void initialEstimate(const OptimizableGraph::VertexSet&,
                       OptimizableGraph::Vertex*) override {
    std::cerr << "initialEstimate() is not implemented, please give "
                 "implementation in your derived class"
              << std::endl;
  }

  /**
   * set the dimension for a dynamically sizeable error function.
   * The member will not be declared for edges having a fixed size at compile
   * time.
   */
  template <int Dim = D>
  typename std::enable_if<Dim == -1, void>::type setDimension(int dim) {
    dimension_ = dim;
    information_.resize(dim, dim);
    error_.resize(dim, 1);
  }

 protected:
  Measurement measurement_;      ///< the measurement of the edge
  InformationType information_;  ///< information matrix of the edge.
                                 ///< Information = inv(covariance)
  ErrorVector error_;  ///< error vector, stores the result after computeError()
                       ///< is called

  /**
   * calculate the robust information matrix by updating the information matrix
   * of the error
   */
  InformationType robustInformation(const Vector3& rho) const {
    InformationType result = rho[1] * information_;
    // ErrorVector weightedError = information_ * error_;
    // result.noalias() += 2 * rho[2] * (weightedError *
    // weightedError.transpose());
    return result;
  }

  //! write the upper trinagular part of the information matrix into the stream
  bool writeInformationMatrix(std::ostream& os) const {
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << information()(i, j) << " ";
    return os.good();
  }
  //! reads the upper triangular part of the matrix and recovers the missing
  //! symmetrical elements
  bool readInformationMatrix(std::istream& is) {
    for (int i = 0; i < information().rows() && is.good(); ++i)
      for (int j = i; j < information().cols() && is.good(); ++j) {
        is >> information()(i, j);
        if (i != j) information()(j, i) = information()(i, j);
      }
    return is.good() || is.eof();
  }
  //! write the param IDs that are potentially used by the edge
  bool writeParamIds(std::ostream& os) const {
    for (auto id : parameterIds_) os << id << " ";
    return os.good();
  }
  //! reads the param IDs from the stream
  bool readParamIds(std::istream& is) {
    for (size_t i = 0; i < numParameters(); ++i) {
      int paramId;
      is >> paramId;
      setParameterId(i, paramId);
    }
    return is.good() || is.eof();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace g2o

#endif
