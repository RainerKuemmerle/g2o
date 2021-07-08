#pragma once

#include <Eigen/Core>

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "glog/logging.h"

namespace g2o {

class UnaryObservationEdge
    : public g2o::BaseUnaryEdge<3, SE3Quat, VertexSE3Expmap> {
 public:
  UnaryObservationEdge(const SE3Quat prior, const Eigen::Matrix3d cov) {
    setMeasurement(prior);
    setInformation(cov.inverse());

    // For full 6-DOF
    // Eigen::Matrix<double, 6, 6> info;
    // info.setZero();
    // info(0, 0) = 100;
    // info(1, 1) = 100;
    // info(2, 2) = 100;
    // info.block<3, 3>(3, 3) = cov.inverse();
    // setInformation(info);

    // For flattening
    // Eigen::Matrix<double, 12, 12> info;
    // info.setZero();
    // info.block<9, 9>(0, 0).setConstant(100);
    // info.block<3, 3>(9, 9) = cov.inverse();
    // setInformation(info);
  }

  void computeError() override {
    const VertexSE3Expmap* vi =
        static_cast<const VertexSE3Expmap*>(_vertices[0]);

    const SE3Quat Tm(_measurement);
    const SE3Quat se3_error = vi->estimate().inverse() * Tm;
    _error = se3_error.log().tail(3);

    // Attempt: Delta q
    // Eigen::Quaterniond delta_q = vi->estimate().inverse().rotation() *
    //                              Tm.inverse().rotation().conjugate();
    // _error.head(3) = 2 * delta_q.vec();

    // Attempt: Flatten R
    // _error.segment<3>(0) = vi->estimate().inverse().rotation().matrix().col(0) -
    //                        Tm.inverse().rotation().matrix().col(0);
    // _error.segment<3>(3) = vi->estimate().inverse().rotation().matrix().col(1) -
    //                        Tm.inverse().rotation().matrix().col(1);
    // _error.segment<3>(6) = vi->estimate().inverse().rotation().matrix().col(2) -
    //                        Tm.inverse().rotation().matrix().col(2);
    // _error.segment<3>(9) =
    //     vi->estimate().inverse().translation() - Tm.inverse().translation();
  }

  void linearizeOplus() override {
    const SE3Quat Tij = _measurement;
    const SE3Quat invTij = Tij.inverse();
    _jacobianOplusXi = -invTij.adj().bottomRows(3);
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

 private:
};

} // namespace g2o
