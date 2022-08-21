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

#ifndef G2O_VERTEX_ELLIPSE_H
#define G2O_VERTEX_ELLIPSE_H

#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_data_api.h"
#include "robot_data.h"

namespace g2o {

/**
 * \brief string ellipse to be attached to a vertex
 */
class G2O_TYPES_DATA_API VertexEllipse : public RobotData {
 public:
  using myVector2fVector =
      std::vector<Vector2F, Eigen::aligned_allocator<Vector2F>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexEllipse();

  bool write(std::ostream& os) const override;
  bool read(std::istream& is) override;

  const Matrix3F& covariance() { return covariance_; }
  void setCovariance(Matrix3F& c) {
    covariance_ = c;
    updateSVD();
  }
  const Matrix2F& U() { return UMatrix_; }
  const Vector2F& singularValues() { return singularValues_; }

  const myVector2fVector& matchingVertices() { return matchingVertices_; }
  void addMatchingVertex(float x, float y) {
    Vector2F v(x, y);
    matchingVertices_.push_back(v);
  }

  void clearMatchingVertices() { matchingVertices_.clear(); }

  const std::vector<int>& matchingVerticesIDs() { return matchingVerticesIDs_; }
  void addMatchingVertexID(int id) { matchingVerticesIDs_.push_back(id); }
  void clearMatchingVerticesIDs() { matchingVerticesIDs_.clear(); }

 protected:
  void updateSVD() const;
  Matrix3F covariance_;
  mutable Matrix2F UMatrix_;
  mutable Vector2F singularValues_;
  std::vector<int> matchingVerticesIDs_;
  myVector2fVector matchingVertices_;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_DATA_API VertexEllipseDrawAction : public DrawAction {
 public:
  VertexEllipseDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<DoubleProperty> scaleFactor_;
};
#endif

}  // namespace g2o

#endif
