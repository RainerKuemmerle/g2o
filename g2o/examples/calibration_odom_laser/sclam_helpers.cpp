// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G.Grisetti, W. Burgard
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

#include "sclam_helpers.h"

#include <iostream>
#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"
#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "gm2dl_io.h"

namespace g2o {

static constexpr double kInformationScalingOdometry = 100;

void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer,
                                       const DataQueue& odomData) {
  auto odomParamsVertex = std::make_shared<VertexOdomDifferentialParams>();
  odomParamsVertex->setToOrigin();
  odomParamsVertex->setId(Gm2dlIO::kIdOdomcalib);
  optimizer.addVertex(odomParamsVertex);

  SparseOptimizer::EdgeSet odomCalibEdges;
  for (const auto& it : optimizer.edges()) {
    auto* scanmatchEdge = dynamic_cast<EdgeSE2SensorCalib*>(it.get());
    if (!scanmatchEdge) continue;

    auto r1 =
        std::dynamic_pointer_cast<VertexSE2>(scanmatchEdge->vertices()[0]);
    auto r2 =
        std::dynamic_pointer_cast<VertexSE2>(scanmatchEdge->vertices()[1]);
    if (r2->id() - r1->id() != 1) {  // ignore non-incremental edges
      continue;
    }

    auto rl1 = std::dynamic_pointer_cast<RobotLaser>(r1->userData());
    auto rl2 = std::dynamic_pointer_cast<RobotLaser>(r2->userData());
    auto odom1 = std::dynamic_pointer_cast<RobotLaser>(
        odomData.findClosestData(rl1->timestamp()));
    auto odom2 = std::dynamic_pointer_cast<RobotLaser>(
        odomData.findClosestData(rl2->timestamp()));

    if (fabs(rl1->timestamp() - rl2->timestamp()) < 1e-7) {
      std::cerr << "strange edge " << r1->id() << " <-> " << r2->id()
                << std::endl;
      std::cerr << FIXED(PVAR(rl1->timestamp())
                         << "\t " << PVAR(rl2->timestamp()))
                << std::endl;
      std::cerr << FIXED(PVAR(odom1->timestamp())
                         << "\t " << PVAR(odom2->timestamp()))
                << std::endl;
    }

    // cerr << PVAR(odom1->odomPose().toVector().transpose()) << endl;

    const SE2 odomMotion = odom1->odomPose().inverse() * odom2->odomPose();
    // cerr << PVAR(odomMotion.toVector().transpose()) << endl;
    // cerr << PVAR(scanmatchEdge->measurement().toVector().transpose()) <<
    // endl;

    auto e = std::make_shared<EdgeSE2OdomDifferentialCalib>();
    e->vertices()[0] = r1;
    e->vertices()[1] = r2;
    e->vertices()[2] = odomParamsVertex;

    const MotionMeasurement mm(
        odomMotion.translation().x(), odomMotion.translation().y(),
        odomMotion.rotation().angle(), odom2->timestamp() - odom1->timestamp());
    e->setMeasurement(OdomConvert::convertToVelocity(mm));
    // cerr << PVAR(e->measurement()) << endl;

    e->information() = Matrix3::Identity() * kInformationScalingOdometry;
    odomCalibEdges.insert(e);
  }

  for (const auto& odomCalibEdge : odomCalibEdges)
    optimizer.addEdge(odomCalibEdge);
}

void allocateSolverForSclam(SparseOptimizer& optimizer, bool levenberg) {
  using SclamBlockSolver = BlockSolver<BlockSolverTraits<-1, -1>>;
  using SclamLinearSolver = LinearSolverEigen<SclamBlockSolver::PoseMatrixType>;

  std::unique_ptr<SclamLinearSolver> linearSolver =
      g2o::make_unique<SclamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithm* solver = nullptr;
  if (levenberg) {
    solver = new OptimizationAlgorithmLevenberg(
        g2o::make_unique<SclamBlockSolver>(std::move(linearSolver)));
  } else {
    solver = new OptimizationAlgorithmGaussNewton(
        g2o::make_unique<SclamBlockSolver>(std::move(linearSolver)));
  }
  optimizer.setAlgorithm(std::unique_ptr<OptimizationAlgorithm>(solver));
}

}  // namespace g2o
