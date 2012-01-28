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

#include "gm2dl_io.h"

#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"
#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/data/data_queue.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <iostream>
using namespace std;

namespace g2o {

  static const double INFORMATION_SCALING_ODOMETRY = 100;

  void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer, const DataQueue& odomData)
  {
    SparseOptimizer::Vertex* odomParamsVertex = 0;
    odomParamsVertex = new VertexOdomDifferentialParams;
    odomParamsVertex->setToOrigin();
    odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
    optimizer.addVertex(odomParamsVertex);

    SparseOptimizer::EdgeSet odomCalibEdges;
    for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      EdgeSE2SensorCalib* scanmatchEdge = dynamic_cast<EdgeSE2SensorCalib*>(*it);
      if (! scanmatchEdge)
        continue;

      VertexSE2* r1 = dynamic_cast<VertexSE2*>(scanmatchEdge->vertices()[0]);
      VertexSE2* r2 = dynamic_cast<VertexSE2*>(scanmatchEdge->vertices()[1]);
      if (r2->id() - r1->id() != 1) { // ignore non-incremental edges
        continue;
      }

      RobotLaser* rl1 = dynamic_cast<RobotLaser*>(r1->userData());
      RobotLaser* rl2 = dynamic_cast<RobotLaser*>(r2->userData());
      RobotLaser* odom1 = dynamic_cast<RobotLaser*>(odomData.findClosestData(rl1->timestamp()));
      RobotLaser* odom2 = dynamic_cast<RobotLaser*>(odomData.findClosestData(rl2->timestamp()));

      if (fabs(rl1->timestamp() - rl2->timestamp()) < 1e-7) {
        cerr << "strange egde " << r1->id() << " <-> " << r2->id() << endl;
        cerr << FIXED(PVAR(rl1->timestamp()) << "\t " << PVAR(rl2->timestamp())) << endl;
        cerr << FIXED(PVAR(odom1->timestamp()) << "\t " << PVAR(odom2->timestamp())) << endl;
      }

      //cerr << PVAR(odom1->odomPose().toVector().transpose()) << endl;

      SE2 odomMotion = odom1->odomPose().inverse() * odom2->odomPose();
      //cerr << PVAR(odomMotion.toVector().transpose()) << endl;
      //cerr << PVAR(scanmatchEdge->measurement().toVector().transpose()) << endl;

      EdgeSE2OdomDifferentialCalib* e = new EdgeSE2OdomDifferentialCalib;
      e->vertices()[0] = r1;
      e->vertices()[1] = r2;
      e->vertices()[2] = odomParamsVertex;

      MotionMeasurement mm(odomMotion.translation().x(), odomMotion.translation().y(), odomMotion.rotation().angle(), odom2->timestamp() - odom1->timestamp());
      e->setMeasurement(OdomConvert::convertToVelocity(mm));
      //cerr << PVAR(e->measurement()) << endl;

      e->information() = Matrix3d::Identity() * INFORMATION_SCALING_ODOMETRY;
      odomCalibEdges.insert(e);

    }

    for (SparseOptimizer::EdgeSet::iterator it = odomCalibEdges.begin(); it != odomCalibEdges.end(); ++it)
      optimizer.addEdge(dynamic_cast<OptimizableGraph::Edge*>(*it));

  }

  void allocateSolverForSclam(SparseOptimizer& optimizer, bool levenberg)
  {
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SclamBlockSolver;
    typedef LinearSolverCSparse<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;

    SclamLinearSolver* linearSolver = new SclamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SclamBlockSolver* blockSolver = new SclamBlockSolver(linearSolver);
    OptimizationAlgorithm* solver = 0;
    if (levenberg) {
      solver = new OptimizationAlgorithmLevenberg(blockSolver);
    } else {
      solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    }
    optimizer.setAlgorithm(solver);
  }
      
} // end namespace
