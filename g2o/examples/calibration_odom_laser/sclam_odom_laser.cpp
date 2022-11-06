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

#include <csignal>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>

#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gm2dl_io.h"
#include "sclam_helpers.h"

using std::cerr;
using std::endl;
using std::string;

static bool hasToStop = false;

void sigquit_handler(int sig) {
  if (sig == SIGINT) {
    hasToStop = true;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << " forcing exit" << endl;
      exit(1);
    }
  }
}

namespace g2o {

static int run_sclam_odom_laser(int argc, char** argv) {
  bool fixLaser;
  int maxIterations;
  bool verbose;
  string inputFilename;
  string outputfilename;
  string rawFilename;
  string odomTestFilename;
  string dumpGraphFilename;
  // command line parsing
  CommandArgs commandLineArguments;
  commandLineArguments.param("i", maxIterations, 10, "perform n iterations");
  commandLineArguments.param("v", verbose, false,
                             "verbose output of the optimization process");
  commandLineArguments.param("o", outputfilename, "",
                             "output final version of the graph");
  commandLineArguments.param("test", odomTestFilename, "",
                             "apply odometry calibration to some test data");
  commandLineArguments.param("dump", dumpGraphFilename, "",
                             "write the graph to the disk");
  commandLineArguments.param("fixLaser", fixLaser, false,
                             "keep the laser offset fixed during optimization");
  commandLineArguments.paramLeftOver("gm2dl-input", inputFilename, "",
                                     "gm2dl file which will be processed");
  commandLineArguments.paramLeftOver("raw-log", rawFilename, "",
                                     "raw log file containing the odometry");

  commandLineArguments.parseArgs(argc, argv);

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

  allocateSolverForSclam(optimizer);

  // loading
  if (!Gm2dlIO::readGm2dl(inputFilename, optimizer, false)) {
    cerr << "Error while loading gm2dl file" << endl;
  }
  DataQueue robotLaserQueue;
  const int numLaserOdom =
      Gm2dlIO::readRobotLaser(rawFilename, robotLaserQueue);
  if (numLaserOdom == 0) {
    cerr << "No raw information read" << endl;
    return 0;
  }
  cerr << "Read " << numLaserOdom << " laser readings from file" << endl;

  const bool gaugeFreedom = optimizer.gaugeFreedom();

  auto gauge = optimizer.findGauge();
  if (gaugeFreedom) {
    if (!gauge) {
      cerr << "# cannot find a vertex to fix in this thing" << endl;
      return 2;
    }
    cerr << "# graph is fixed by node " << gauge->id() << endl;
    gauge->setFixed(true);

  } else {
    cerr << "# graph is fixed by priors" << endl;
  }

  addOdometryCalibLinksDifferential(optimizer, robotLaserQueue);

  // sanity check
  auto dummy = std::shared_ptr<HyperGraph>(&optimizer, [](HyperGraph*) {});
  HyperDijkstra d(dummy);
  UniformCostFunction f;
  d.shortestPaths(gauge, f);
  // cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size() != optimizer.vertices().size()) {
    cerr << CL_RED("Warning: d.visited().size() != optimizer.vertices().size()")
         << endl;
    cerr << "visited: " << d.visited().size() << endl;
    cerr << "vertices: " << optimizer.vertices().size() << endl;
    for (const auto& it : optimizer.vertices()) {
      if (d.visited().count(it.second) == 0) {
        auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
        cerr << "\t unvisited vertex " << it.first << " "
             << static_cast<void*>(v) << endl;
        v->setFixed(true);
      }
    }
  }

  for (const auto& it : optimizer.vertices()) {
    auto* v = static_cast<OptimizableGraph::Vertex*>(it.second.get());
    if (v->fixed()) {
      cerr << "\t fixed vertex " << it.first << endl;
    }
  }

  auto laserOffset = std::dynamic_pointer_cast<VertexSE2>(
      optimizer.vertex(Gm2dlIO::kIdLaserpose));
  auto odomParamsVertex =
      std::dynamic_pointer_cast<VertexOdomDifferentialParams>(
          optimizer.vertex(Gm2dlIO::kIdOdomcalib));

  if (fixLaser) {
    cerr << "Fix position of the laser offset" << endl;
    laserOffset->setFixed(true);
  }

  signal(SIGINT, sigquit_handler);
  cerr << "Doing full estimation" << endl;
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  const int i = optimizer.optimize(maxIterations);
  if (maxIterations > 0 && !i) {
    cerr << "optimize failed, result might be invalid" << endl;
  }

  if (laserOffset) {
    cerr << "Calibrated laser offset (x, y, theta):"
         << laserOffset->estimate().toVector().transpose() << endl;
  }

  if (odomParamsVertex) {
    cerr << "Odometry parameters (scaling factors (v_l, v_r, b)): "
         << odomParamsVertex->estimate().transpose() << endl;
  }

  cerr << "vertices: " << optimizer.vertices().size() << endl;
  cerr << "edges: " << optimizer.edges().size() << endl;

  if (!dumpGraphFilename.empty()) {
    cerr << "Writing " << dumpGraphFilename << " ... ";
    optimizer.save(dumpGraphFilename.c_str());
    cerr << "done." << endl;
  }

  // optional input of a separate file for applying the odometry calibration
  if (!odomTestFilename.empty()) {
    DataQueue testRobotLaserQueue;
    const int numTestOdom =
        Gm2dlIO::readRobotLaser(odomTestFilename, testRobotLaserQueue);
    if (numTestOdom == 0) {
      cerr << "Unable to read test data" << endl;
    } else {
      std::ofstream rawStream("odometry_raw.txt");
      std::ofstream calibratedStream("odometry_calibrated.txt");
      Vector3 odomCalib =
          odomParamsVertex ? odomParamsVertex->estimate() : Vector3::Ones();
      auto prev = std::dynamic_pointer_cast<RobotLaser>(
          testRobotLaserQueue.buffer().begin()->second);
      SE2 prevCalibratedPose = prev->odomPose();

      for (const auto& it : testRobotLaserQueue.buffer()) {
        auto cur = std::dynamic_pointer_cast<RobotLaser>(it.second);
        assert(cur);

        const double dt = cur->timestamp() - prev->timestamp();
        const SE2 motion = prev->odomPose().inverse() * cur->odomPose();

        // convert to velocity Measurement
        const MotionMeasurement motionMeasurement(
            motion.translation().x(), motion.translation().y(),
            motion.rotation().angle(), dt);
        const VelocityMeasurement velocityMeasurement =
            OdomConvert::convertToVelocity(motionMeasurement);

        // apply calibration
        VelocityMeasurement calibratedVelocityMeasurement = velocityMeasurement;
        calibratedVelocityMeasurement.setVl(odomCalib(0) *
                                           calibratedVelocityMeasurement.vl());
        calibratedVelocityMeasurement.setVr(odomCalib(1) *
                                           calibratedVelocityMeasurement.vr());
        const MotionMeasurement mm = OdomConvert::convertToMotion(
            calibratedVelocityMeasurement, odomCalib(2));

        // combine calibrated odometry with the previous pose
        SE2 remappedOdom;
        remappedOdom.fromVector(mm.measurement());
        const SE2 calOdomPose = prevCalibratedPose * remappedOdom;

        // write output
        rawStream << prev->odomPose().translation().x() << " "
                  << prev->odomPose().translation().y() << " "
                  << prev->odomPose().rotation().angle() << endl;
        calibratedStream << calOdomPose.translation().x() << " "
                         << calOdomPose.translation().y() << " "
                         << calOdomPose.rotation().angle() << endl;

        prevCalibratedPose = calOdomPose;
        prev = cur;
      }
    }
  }

  if (!outputfilename.empty()) {
    Gm2dlIO::updateLaserData(optimizer);
    cerr << "Writing " << outputfilename << " ... ";
    const bool writeStatus = Gm2dlIO::writeGm2dl(outputfilename, optimizer);
    cerr << (writeStatus ? "done." : "failed") << endl;
  }

  return 0;
}
}  // namespace g2o

int main(int argc, char** argv) {
  return g2o::run_sclam_odom_laser(argc, argv);
}
