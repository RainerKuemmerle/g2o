#ifndef G2O_TARGET_TYPES_3D_HPP_
#define G2O_TARGET_TYPES_3D_HPP_

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <Eigen/Core>

// This header file specifies a set of types for the different
// tracking examples; note that

class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  VertexPosition3D() = default;

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update;
  }
};

// Store velocity separately from position?
class VertexVelocity3D : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  VertexVelocity3D() = default;

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update;
  }
};

// The idealised GPS measurement; this is 3D and linear
class GPSObservationPosition3DEdge
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPosition3D> {
 public:
  GPSObservationPosition3DEdge() = default;

  void computeError() override {
    const VertexPosition3D* v = vertexXnRaw<0>();
    error_ = v->estimate() - measurement_;
  }
};

#endif  //  __TARGET_TYPES_3D_HPP__
