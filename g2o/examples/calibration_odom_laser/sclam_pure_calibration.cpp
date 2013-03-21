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

#include <iostream>
#include <map>
#include <csignal>
#include <fstream>

#include "sclam_helpers.h"
#include "gm2dl_io.h"
#include "motion_information.h"
#include "closed_form_calibration.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "g2o/types/sclam2d/odometry_measurement.h"
#include "edge_se2_pure_calib.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/data/data_queue.h"

using namespace std;
using namespace g2o;

static Eigen::Vector2d linearSolution;

class VertexBaseline : public BaseVertex<1, double>
{
  public:
    VertexBaseline() {}

    virtual void setToOriginImpl() { _estimate = 1.;}
    virtual void oplusImpl(const double* update) { _estimate += update[0];}
    virtual bool read(std::istream&) { return false;}
    virtual bool write(std::ostream&) const { return false;}
};

class EdgeCalib : public BaseBinaryEdge<3, OdomAndLaserMotion, VertexSE2, VertexBaseline>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeCalib() {}

    void computeError()
    {
      const VertexSE2* laserOffset = static_cast<const VertexSE2*>(_vertices[0]);
      const VertexBaseline* odomParams = dynamic_cast<const VertexBaseline*>(_vertices[1]);

      // get the calibrated motion given by the odometry
      double rl = - odomParams->estimate() * linearSolution(0);
      double rr = odomParams->estimate() * linearSolution(1);
      VelocityMeasurement calibratedVelocityMeasurment(measurement().velocityMeasurement.vl() * rl,
          measurement().velocityMeasurement.vr() * rr,
          measurement().velocityMeasurement.dt());
      MotionMeasurement mm = OdomConvert::convertToMotion(calibratedVelocityMeasurment, odomParams->estimate());
      SE2 Ku_ij;
      Ku_ij.fromVector(mm.measurement());

      SE2 laserMotionInRobotFrame = laserOffset->estimate() * measurement().laserMotion * laserOffset->estimate().inverse();
      SE2 delta = Ku_ij.inverse() * laserMotionInRobotFrame;
      _error = delta.toVector();
    }

    virtual bool read(std::istream&) { return false;}
    virtual bool write(std::ostream&) const { return false;}
};

int main(int argc, char** argv)
{
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
  commandLineArguments.param("v", verbose, false, "verbose output of the optimization process");
  commandLineArguments.param("o", outputfilename, "", "output final version of the graph");
  commandLineArguments.param("test", odomTestFilename, "", "apply odometry calibration to some test data");
  commandLineArguments.param("dump", dumpGraphFilename, "", "write the graph to the disk");
  commandLineArguments.param("fixLaser", fixLaser, false, "keep the laser offset fixed during optimization");
  commandLineArguments.paramLeftOver("gm2dl-input", inputFilename, "", "gm2dl file which will be processed");
  commandLineArguments.paramLeftOver("raw-log", rawFilename, "", "raw log file containing the odometry");

  commandLineArguments.parseArgs(argc, argv);

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);

  allocateSolverForSclam(optimizer);

  // loading
  DataQueue odometryQueue;
  int numLaserOdom = Gm2dlIO::readRobotLaser(rawFilename, odometryQueue);
  if (numLaserOdom == 0) {
    cerr << "No raw information read" << endl;
    return 0;
  }
  cerr << "Read " << numLaserOdom << " laser readings from file" << endl;

  Eigen::Vector3d odomCalib(1., 1., 1.);
  SE2 initialLaserPose;
  DataQueue robotLaserQueue;
  int numRobotLaser = Gm2dlIO::readRobotLaser(inputFilename, robotLaserQueue);
  if (numRobotLaser == 0) {
    cerr << "No robot laser read" << endl;
    return 0;
  } else {
    RobotLaser* rl = dynamic_cast<RobotLaser*>(robotLaserQueue.buffer().begin()->second);
    initialLaserPose = rl->odomPose().inverse() * rl->laserPose();
    cerr << PVAR(initialLaserPose.toVector().transpose()) << endl;
  }

  // adding the measurements
  vector<MotionInformation, Eigen::aligned_allocator<MotionInformation> > motions;
  {
    std::map<double, RobotData*>::const_iterator it = robotLaserQueue.buffer().begin();
    std::map<double, RobotData*>::const_iterator prevIt = it++;
    for (; it != robotLaserQueue.buffer().end(); ++it) {
      MotionInformation mi;
      RobotLaser* prevLaser = dynamic_cast<RobotLaser*>(prevIt->second);
      RobotLaser* curLaser = dynamic_cast<RobotLaser*>(it->second);
      mi.laserMotion = prevLaser->laserPose().inverse() * curLaser->laserPose();
      // get the motion of the robot in that time interval
      RobotLaser* prevOdom = dynamic_cast<RobotLaser*>(odometryQueue.findClosestData(prevLaser->timestamp()));
      RobotLaser* curOdom = dynamic_cast<RobotLaser*>(odometryQueue.findClosestData(curLaser->timestamp()));
      mi.odomMotion = prevOdom->odomPose().inverse() * curOdom->odomPose();
      mi.timeInterval = prevOdom->timestamp() - curOdom->timestamp();
      prevIt = it;
      motions.push_back(mi);
    }
  }

  if (1) {
    VertexSE2* laserOffset = new VertexSE2;
    laserOffset->setId(Gm2dlIO::ID_LASERPOSE);
    laserOffset->setEstimate(initialLaserPose);
    optimizer.addVertex(laserOffset);
    VertexOdomDifferentialParams* odomParamsVertex = new VertexOdomDifferentialParams;
    odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
    odomParamsVertex->setEstimate(Eigen::Vector3d(1., 1., 1.));
    optimizer.addVertex(odomParamsVertex);
    for (size_t i = 0; i < motions.size(); ++i) {
      const SE2& odomMotion = motions[i].odomMotion;
      const SE2& laserMotion = motions[i].laserMotion;
      const double& timeInterval = motions[i].timeInterval;
      // add the edge
      MotionMeasurement mm(odomMotion.translation().x(), odomMotion.translation().y(), odomMotion.rotation().angle(), timeInterval);
      OdomAndLaserMotion meas;
      meas.velocityMeasurement = OdomConvert::convertToVelocity(mm);
      meas.laserMotion = laserMotion;
      EdgeSE2PureCalib* calibEdge = new EdgeSE2PureCalib;
      calibEdge->setVertex(0, laserOffset);
      calibEdge->setVertex(1, odomParamsVertex);
      calibEdge->setInformation(Eigen::Matrix3d::Identity());
      calibEdge->setMeasurement(meas);
      if (! optimizer.addEdge(calibEdge)) {
        cerr << "Error adding calib edge" << endl;
        delete calibEdge;
      }
    }

    if (fixLaser) {
      cerr << "Fix position of the laser offset" << endl;
      laserOffset->setFixed(true);
    }

    cerr << "\nPerforming full non-linear estimation" << endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(maxIterations);
    cerr << "Calibrated laser offset (x, y, theta):" << laserOffset->estimate().toVector().transpose() << endl;
    odomCalib = odomParamsVertex->estimate();
    cerr << "Odometry parameters (scaling factors (v_l, v_r, b)): " << odomParamsVertex->estimate().transpose() << endl;
    optimizer.clear();
  }

  // linear least squares for some parameters
  {
    Eigen::MatrixXd A(motions.size(), 2);
    Eigen::VectorXd x(motions.size());
    for (size_t i = 0; i < motions.size(); ++i) {
      const SE2& odomMotion = motions[i].odomMotion;
      const SE2& laserMotion = motions[i].laserMotion;
      const double& timeInterval = motions[i].timeInterval;
      MotionMeasurement mm(odomMotion.translation().x(), odomMotion.translation().y(), odomMotion.rotation().angle(), timeInterval);
      VelocityMeasurement velMeas = OdomConvert::convertToVelocity(mm);
      A(i, 0) = velMeas.vl() * timeInterval;
      A(i, 1) = velMeas.vr() * timeInterval;
      x(i) = laserMotion.rotation().angle();
    }
    //linearSolution = (A.transpose() * A).inverse() * A.transpose() * x;
    linearSolution = A.colPivHouseholderQr().solve(x);
    //cout << PVAR(linearSolution.transpose()) << endl;
  }

  //constructing non-linear least squares
  VertexSE2* laserOffset = new VertexSE2;
  laserOffset->setId(Gm2dlIO::ID_LASERPOSE);
  laserOffset->setEstimate(initialLaserPose);
  optimizer.addVertex(laserOffset);
  VertexBaseline* odomParamsVertex = new VertexBaseline;
  odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
  odomParamsVertex->setEstimate(1.);
  optimizer.addVertex(odomParamsVertex);
  for (size_t i = 0; i < motions.size(); ++i) {
    const SE2& odomMotion = motions[i].odomMotion;
    const SE2& laserMotion = motions[i].laserMotion;
    const double& timeInterval = motions[i].timeInterval;
    // add the edge
    MotionMeasurement mm(odomMotion.translation().x(), odomMotion.translation().y(), odomMotion.rotation().angle(), timeInterval);
    OdomAndLaserMotion meas;
    meas.velocityMeasurement = OdomConvert::convertToVelocity(mm);
    meas.laserMotion = laserMotion;
    EdgeCalib* calibEdge = new EdgeCalib;
    calibEdge->setVertex(0, laserOffset);
    calibEdge->setVertex(1, odomParamsVertex);
    calibEdge->setInformation(Eigen::Matrix3d::Identity());
    calibEdge->setMeasurement(meas);
    if (! optimizer.addEdge(calibEdge)) {
      cerr << "Error adding calib edge" << endl;
      delete calibEdge;
    }
  }

  if (fixLaser) {
    cerr << "Fix position of the laser offset" << endl;
    laserOffset->setFixed(true);
  }

  cerr << "\nPerforming partial non-linear estimation" << endl;
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(maxIterations);
  cerr << "Calibrated laser offset (x, y, theta):" << laserOffset->estimate().toVector().transpose() << endl;
  odomCalib(0) = -1. * linearSolution(0) * odomParamsVertex->estimate();
  odomCalib(1) = linearSolution(1) * odomParamsVertex->estimate();
  odomCalib(2) = odomParamsVertex->estimate();
  cerr << "Odometry parameters (scaling factors (v_l, v_r, b)): " << odomCalib.transpose() << endl;

  {
    SE2 closedFormLaser;
    Eigen::Vector3d closedFormOdom;
    ClosedFormCalibration::calibrate(motions, closedFormLaser, closedFormOdom);
    cerr << "\nObtaining closed form solution" << endl;
    cerr << "Calibrated laser offset (x, y, theta):" << closedFormLaser.toVector().transpose() << endl;
    cerr << "Odometry parameters (scaling factors (v_l, v_r, b)): " << closedFormOdom.transpose() << endl;
  }

  if (dumpGraphFilename.size() > 0) {
    cerr << "Writing " << dumpGraphFilename << " ... ";
    optimizer.save(dumpGraphFilename.c_str());
    cerr << "done." << endl;
  }

  // optional input of a separate file for applying the odometry calibration
  if (odomTestFilename.size() > 0) {

    DataQueue testRobotLaserQueue;
    int numTestOdom = Gm2dlIO::readRobotLaser(odomTestFilename, testRobotLaserQueue);
    if (numTestOdom == 0) {
      cerr << "Unable to read test data" << endl;
    } else {

      ofstream rawStream("odometry_raw.txt");
      ofstream calibratedStream("odometry_calibrated.txt");
      RobotLaser* prev = dynamic_cast<RobotLaser*>(testRobotLaserQueue.buffer().begin()->second);
      SE2 prevCalibratedPose = prev->odomPose();

      for (DataQueue::Buffer::const_iterator it = testRobotLaserQueue.buffer().begin(); it != testRobotLaserQueue.buffer().end(); ++it) {
        RobotLaser* cur = dynamic_cast<RobotLaser*>(it->second);
        assert(cur);

        double dt = cur->timestamp() - prev->timestamp();
        SE2 motion = prev->odomPose().inverse() * cur->odomPose();

        // convert to velocity measurement
        MotionMeasurement motionMeasurement(motion.translation().x(), motion.translation().y(), motion.rotation().angle(), dt);
        VelocityMeasurement velocityMeasurement = OdomConvert::convertToVelocity(motionMeasurement);

        // apply calibration
        VelocityMeasurement calibratedVelocityMeasurment = velocityMeasurement;
        calibratedVelocityMeasurment.setVl(odomCalib(0) * calibratedVelocityMeasurment.vl());
        calibratedVelocityMeasurment.setVr(odomCalib(1) * calibratedVelocityMeasurment.vr());
        MotionMeasurement mm = OdomConvert::convertToMotion(calibratedVelocityMeasurment, odomCalib(2));

        // combine calibrated odometry with the previous pose
        SE2 remappedOdom;
        remappedOdom.fromVector(mm.measurement());
        SE2 calOdomPose = prevCalibratedPose * remappedOdom;

        // write output
        rawStream << prev->odomPose().translation().x() << " " << prev->odomPose().translation().y() << " " << prev->odomPose().rotation().angle() << endl;
        calibratedStream << calOdomPose.translation().x() << " " << calOdomPose.translation().y() << " " << calOdomPose.rotation().angle() << endl;

        prevCalibratedPose = calOdomPose;
        prev = cur;
      }
    }

  }

  return 0;
}
