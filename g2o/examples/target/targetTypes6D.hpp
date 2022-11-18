#ifndef G2O_TARGET_TYPES_6D_HPP_
#define G2O_TARGET_TYPES_6D_HPP_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <Eigen/Core>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// This header file specifies a set of types for the different
// tracking examples; note that

class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPosition3D() = default;

  void setToOriginImpl() override { estimate_.setZero(); }

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

class PositionVelocity3DEdge {};

class VertexPositionVelocity3D : public g2o::BaseVertex<6, Vector6d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPositionVelocity3D() = default;

  void setToOriginImpl() override { estimate_.setZero(); }

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

// The odometry which links pairs of nodes together
class TargetOdometry3DEdge
    : public g2o::BaseBinaryEdge<6, Eigen::Vector3d, VertexPositionVelocity3D,
                                 VertexPositionVelocity3D> {
 public:
  TargetOdometry3DEdge(double dt, double noiseSigma) {
    dt_ = dt;

    double q = noiseSigma * noiseSigma;
    double dt2 = dt * dt;

    // Process noise covariance matrix; this assumes an "impulse"
    // noise model; we add a small stabilising term on the diagonal to make it
    // invertible
    Matrix6d Q = Matrix6d::Zero();
    Q(0, 0) = Q(1, 1) = Q(2, 2) = dt2 * dt2 * q / 4 + 1e-4;
    Q(0, 3) = Q(1, 4) = Q(2, 5) = dt * dt2 * q / 2;
    Q(3, 3) = Q(4, 4) = Q(5, 5) = dt2 * q + 1e-4;
    Q(3, 0) = Q(4, 1) = Q(5, 2) = dt * dt2 * q / 2;

    setInformation(Q.inverse());
  }

  /** set the estimate of the to vertex, based on the estimate of the from
   * vertex in the edge. */
  void initialEstimate(const g2o::OptimizableGraph::VertexSet& from,
                       g2o::OptimizableGraph::Vertex* to) override {
    assert(from.size() == 1);
    const VertexPositionVelocity3D* vi =
        static_cast<const VertexPositionVelocity3D*>(from.begin()->get());
    auto* vj = static_cast<VertexPositionVelocity3D*>(to);
    Vector6d viEst = vi->estimate();
    Vector6d vjEst = viEst;

    for (int m = 0; m < 3; m++) {
      vjEst[m] += dt_ * (vjEst[m + 3] + 0.5 * dt_ * measurement_[m]);
    }

    for (int m = 0; m < 3; m++) {
      vjEst[m + 3] += dt_ * measurement_[m];
    }
    vj->setEstimate(vjEst);
  }

  /** override in your class if it's not possible to initialize the vertices in
   * certain combinations
   */
  double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& from,
                                 g2o::OptimizableGraph::Vertex* to) override {
    // only works on sequential vertices
    const VertexPositionVelocity3D* vi =
        static_cast<const VertexPositionVelocity3D*>(from.begin()->get());
    return (to->id() - vi->id() == 1) ? 1.0 : -1.0;
  }

  void computeError() override {
    const VertexPositionVelocity3D* vi = vertexXnRaw<0>();
    const VertexPositionVelocity3D* vj = vertexXnRaw<1>();

    for (int k = 0; k < 3; k++) {
      error_[k] = vi->estimate()[k] +
                  dt_ * (vi->estimate()[k + 3] + 0.5 * dt_ * measurement_[k]) -
                  vj->estimate()[k];
    }
    for (int k = 3; k < 6; k++) {
      error_[k] =
          vi->estimate()[k] + dt_ * measurement_[k - 3] - vj->estimate()[k];
    }
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

 private:
  double dt_;
};

// The GPS
class GPSObservationEdgePositionVelocity3D
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D> {
 public:
  GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement,
                                       double noiseSigma) {
    setMeasurement(measurement);
    setInformation(Eigen::Matrix3d::Identity() / (noiseSigma * noiseSigma));
  }

  void computeError() override {
    const VertexPositionVelocity3D* v = vertexXnRaw<0>();
    error_ = v->estimate().head<3>() - measurement_;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

#endif  //  __TARGET_TYPES_6D_HPP__
