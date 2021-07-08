#pragma once

#include <Eigen/Core>

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "types_six_dof_expmap.h"
#include "glog/logging.h"

namespace g2o {

class BinaryObservationEdge : public g2o::BaseBinaryEdge<1, double, VertexSE3Expmap, VertexSE3Expmap>
{
public:
  BinaryObservationEdge(const double prior, double noiseSigma = 0.1)
  {
    setMeasurement(prior);
    setInformation(Eigen::Matrix<double, 1, 1>::Identity() /
                   std::pow(noiseSigma, 2));
  }

  void computeError() override
  {
    const VertexSE3Expmap * from = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    CHECK_NOTNULL(from);
    const VertexSE3Expmap * to = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    CHECK_NOTNULL(to);

    const Vector3d estimate =
        (to->estimate().inverse() * from->estimate()).translation();

    _error.setZero();
    _error[0] = estimate.norm() - _measurement;
    LOG(INFO) << _error.transpose();
  }

  void linearizeOplus() override {
    const VertexSE3Expmap * from = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    CHECK_NOTNULL(from);
    const VertexSE3Expmap * to = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    CHECK_NOTNULL(to);

    const Vector3d estimate =
        (to->estimate().inverse() * from->estimate()).translation();
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<1, 3>(0, 0) =
        -estimate.transpose() / estimate.norm();
    _jacobianOplusXj.setZero();
    _jacobianOplusXj.block<1, 3>(0, 0) =
        estimate.transpose() / estimate.norm();
  }

  bool read(std::istream& /*is*/) override
  {
    return false;
  }
  
  bool write(std::ostream& /*os*/) const override
  {
    return false;
  }
};

}
