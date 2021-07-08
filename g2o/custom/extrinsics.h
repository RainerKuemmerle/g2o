#pragma once

#include <Eigen/Core>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "glog/logging.h"

namespace g2o {

class ExtrinsicsEdge
    : public g2o::BaseBinaryEdge<3, SE3Quat, VertexSE3Expmap, VertexSE3Expmap> {
 public:
  ExtrinsicsEdge(const SE3Quat untransformed_prior, const Eigen::MatrixXd cov) {
    setMeasurement(untransformed_prior);
    setInformation(cov.inverse());
  }

  void computeError() override {
    // One end is at the extrinsics
    const VertexSE3Expmap* vertex_extrinsics =
        static_cast<const VertexSE3Expmap*>(_vertices[0]);

    // Other end is at the camera pose
    const VertexSE3Expmap* vertex_cam =
        static_cast<const VertexSE3Expmap*>(_vertices[1]);

    const SE3Quat C(_measurement);
    // To_c * Tc_l
    const SE3Quat To_l = vertex_extrinsics->estimate() * vertex_cam->estimate();
    const SE3Quat se3_error = To_l.inverse() * C;

    auto error = se3_error.log();
    _error = error.tail(3);
  }

  void linearizeOplus() override {
    const VertexSE3Expmap* vi = static_cast<VertexSE3Expmap*>(_vertices[0]);
    const SE3Quat Ti(vi->estimate());

    const SE3Quat& Tij = _measurement;
    const SE3Quat invTij = Tij.inverse();

    const SE3Quat invTij_Ti = invTij * Ti;

    _jacobianOplusXi = -invTij.adj().bottomRows(3);
    _jacobianOplusXi.col(5).setZero();
    _jacobianOplusXj = -invTij_Ti.adj().bottomRows(3);
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

 private:
};

} // namespace g2o
