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

#include "graph.pb.h"

namespace g2o {

namespace internal {

#ifdef G2O_OPENMP
struct QuadraticFormLock {
  explicit QuadraticFormLock(OptimizableGraph::Vertex& vertex) : _vertex(vertex) {
    _vertex.lockQuadraticForm();
  }
  ~QuadraticFormLock() { _vertex.unlockQuadraticForm(); }

 private:
  OptimizableGraph::Vertex& _vertex;
};
#else
struct QuadraticFormLock {
  explicit QuadraticFormLock(OptimizableGraph::Vertex&) {}
  ~QuadraticFormLock() {}
};
#endif

/**
 * Declaring the types for the error vector and the information matrix depending on the size of the
 * error function. In particular, the information matrix needs to match the size of the error
 * vector.
 */
template <int D>
struct BaseEdgeTraits {
  static constexpr int Dimension = D;
  typedef Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<number_t, D, D, Eigen::ColMajor> InformationType;
};
/**
 * Same as above but for dimension not known at compilation, i.e., dynamically sized edges.
 */
template <>
struct BaseEdgeTraits<-1> {
  static constexpr int Dimension = -1;
  typedef Eigen::Matrix<number_t, Eigen::Dynamic, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<number_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> InformationType;
};

}  // namespace internal

template <int D, typename E>
class BaseEdge : public OptimizableGraph::Edge {
 public:
  static constexpr int Dimension = internal::BaseEdgeTraits<D>::Dimension;
  typedef E Measurement;
  typedef typename internal::BaseEdgeTraits<D>::ErrorVector ErrorVector;
  typedef typename internal::BaseEdgeTraits<D>::InformationType InformationType;

  BaseEdge() : OptimizableGraph::Edge() { _dimension = D; }

  virtual ~BaseEdge() {}

  virtual number_t chi2() const { return _error.dot(information() * _error); }

  virtual const number_t* errorData() const { return _error.data(); }
  virtual number_t* errorData() { return _error.data(); }
  const ErrorVector& error() const { return _error; }
  ErrorVector& error() { return _error; }

  //! information matrix of the constraint
  EIGEN_STRONG_INLINE const InformationType& information() const { return _information; }
  EIGEN_STRONG_INLINE InformationType& information() { return _information; }
  void setInformation(const InformationType& information) { _information = information; }

  virtual const number_t* informationData() const { return _information.data(); }
  virtual number_t* informationData() { return _information.data(); }

  //! accessor functions for the measurement represented by the edge
  EIGEN_STRONG_INLINE const Measurement& measurement() const { return _measurement; }
  virtual void setMeasurement(const Measurement& m) { _measurement = m; }

  virtual int rank() const { return _dimension; }

  virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*) {
    std::cerr << "inititialEstimate() is not implemented, please give implementation in your "
                 "derived class"
              << std::endl;
  }

  /**
   * set the dimension for a dynamically sizeable error function.
   * The member will not be declared for edges having a fixed size at compile time.
   */
  template <int Dim = D>
  typename std::enable_if<Dim == -1, void>::type setDimension(int dim) {
    _dimension = dim;
    _information.resize(dim, dim);
    _error.resize(dim, 1);
  }

  virtual bool readProto(const g2o::proto::Row&) { return false; }
  virtual bool writeProto(g2o::proto::Row*) const { return false; }

 protected:
  Measurement _measurement;      ///< the measurement of the edge
  InformationType _information;  ///< information matrix of the edge. Information = inv(covariance)
  ErrorVector _error;            ///< error vector, stores the result after computeError() is called

  /**
   * calculate the robust information matrix by updating the information matrix of the error
   */
  InformationType robustInformation(const Vector3& rho) const {
    InformationType result = rho[1] * _information;
    // ErrorVector weightedErrror = _information * _error;
    // result.noalias() += 2 * rho[2] * (weightedErrror * weightedErrror.transpose());
    return result;
  }

  //! write the upper trinagular part of the information matrix into the stream
  bool writeInformationMatrix(std::ostream& os) const {
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) os << information()(i, j) << " ";
    return os.good();
  }
  bool writeInformationMatrixProto(g2o::proto::Row* row) const {
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        row->add_value(information()(i, j));
      }
    }
    return true;
  }
  //! reads the upper triangular part of the matrix and recovers the missing symmetrical elements
  bool readInformationMatrix(std::istream& is) {
    for (int i = 0; i < information().rows() && is.good(); ++i)
      for (int j = i; j < information().cols() && is.good(); ++j) {
        is >> information()(i, j);
        if (i != j) information()(j, i) = information()(i, j);
      }
    return is.good() || is.eof();
  }
  bool readInformationMatrixProto(int idx, const g2o::proto::Row& row) {
    int advance = idx;
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        if (advance < row.value_size()) {
          information()(i, j) = row.value(advance);
          advance = advance + 1;
          if (i != j) {
            information()(j, i) = information()(i, j);
          }
        }
      }
    }
    return true;
  }

  //! write the param IDs that are potentially used by the edge
  bool writeParamIds(std::ostream& os) const {
    for (auto id : _parameterIds) os << id << " ";
    return os.good();
  }

  bool writeParamIdsProto(g2o::proto::Row* row) const {
    for (auto id : _parameterIds) {
      row->add_param(id);
    }
    return true;
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

  bool readParamIdsProto(const g2o::proto::Row& row) {
    for (size_t i = 0; i < numParameters() && i < ((size_t) row.param_size()); ++i) {
      int paramId = row.param(i);
      setParameterId(i, paramId);
    }
    return true;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace g2o

#endif
