// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "simulator2d.h"

using std::cerr;
using std::endl;

int main(int argc, char** argv) {
  g2o::CommandArgs arg;
  int nlandmarks;
  int simSteps;
  double worldSize;
  bool hasOdom;
  bool hasPoseSensor;
  bool hasPointSensor;
  bool hasPointBearingSensor;
  bool hasCompass;
  bool hasGPS;

  bool hasSegmentSensor;
  int nSegments;
  int segmentGridSize;
  double minSegmentLength;
  double maxSegmentLength;

  std::string outputFilename;
  arg.param("nlandmarks", nlandmarks, 100, "number of landmarks in the map");
  arg.param("nSegments", nSegments, 1000, "number of segments");
  arg.param("segmentGridSize", segmentGridSize, 50,
            "number of cells of the grid where to align the segments");
  arg.param("minSegmentLength", minSegmentLength, 0.5,
            "minimal Length of a segment in the world");
  arg.param("maxSegmentLength", maxSegmentLength, 3,
            "maximal Length of a segment in the world");

  arg.param("simSteps", simSteps, 100, "number of simulation steps");
  arg.param("worldSize", worldSize, 25.0, "size of the world");
  arg.param("hasOdom", hasOdom, false, "the robot has an odometry");
  arg.param("hasPointSensor", hasPointSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointBearingSensor", hasPointBearingSensor, false,
            "the robot has a point bearing sensor");
  arg.param("hasPoseSensor", hasPoseSensor, false,
            "the robot has a pose sensor");
  arg.param("hasCompass", hasCompass, false, "the robot has a compass");
  arg.param("hasGPS", hasGPS, false, "the robot has a GPS");
  arg.param("hasSegmentSensor", hasSegmentSensor, false,
            "the robot has a segment sensor");
  arg.paramLeftOver("graph-output", outputFilename, "simulator_out.g2o",
                    "graph file which will be written", true);

  arg.parseArgs(argc, argv);

  std::mt19937 generator;
  g2o::OptimizableGraph graph;
  g2o::World world(&graph);
  for (int i = 0; i < nlandmarks; i++) {
    auto* landmark = new g2o::WorldObjectPointXY;
    double x = g2o::sampleUniform(-.5, .5, &generator) * (worldSize + 5);
    double y = g2o::sampleUniform(-.5, .5, &generator) * (worldSize + 5);
    landmark->vertex()->setEstimate(g2o::Vector2(x, y));
    world.addWorldObject(landmark);
  }

  cerr << "nSegments = " << nSegments << endl;

  for (int i = 0; i < nSegments; i++) {
    auto* segment = new g2o::WorldObjectSegment2D;
    int ix = g2o::sampleUniform(-segmentGridSize, segmentGridSize, &generator);
    int iy = g2o::sampleUniform(-segmentGridSize, segmentGridSize, &generator);
    int ith = g2o::sampleUniform(0, 3, &generator);
    double th = (M_PI / 2) * ith;
    th = atan2(sin(th), cos(th));
    double xc = ix * (worldSize / segmentGridSize);
    double yc = iy * (worldSize / segmentGridSize);

    double l2 =
        g2o::sampleUniform(minSegmentLength, maxSegmentLength, &generator);

    double x1 = xc + cos(th) * l2;
    double y1 = yc + sin(th) * l2;
    double x2 = xc - cos(th) * l2;
    double y2 = yc - sin(th) * l2;

    segment->vertex()->setEstimateP1(g2o::Vector2(x1, y1));
    segment->vertex()->setEstimateP2(g2o::Vector2(x2, y2));
    world.addWorldObject(segment);
  }

  g2o::Robot2D robot(&world, "myRobot");
  world.addRobot(&robot);

  std::stringstream ss;
  ss << "-ws" << worldSize;
  ss << "-nl" << nlandmarks;
  ss << "-steps" << simSteps;

  if (hasOdom) {
    auto* odometrySensor = new g2o::SensorOdometry2D("odometry");
    robot.addSensor(odometrySensor);
    g2o::Matrix3 odomInfo = odometrySensor->information();
    odomInfo.setIdentity();
    odomInfo *= 500;
    odomInfo(2, 2) = 5000;
    odometrySensor->setInformation(odomInfo);
    ss << "-odom";
  }

  if (hasPoseSensor) {
    auto* poseSensor = new g2o::SensorPose2D("poseSensor");
    robot.addSensor(poseSensor);
    g2o::Matrix3 poseInfo = poseSensor->information();
    poseInfo.setIdentity();
    poseInfo *= 500;
    poseInfo(2, 2) = 5000;
    poseSensor->setInformation(poseInfo);
    ss << "-pose";
  }

  if (hasPointSensor) {
    auto* pointSensor = new g2o::SensorPointXY("pointSensor");
    robot.addSensor(pointSensor);
    g2o::Matrix2 pointInfo = pointSensor->information();
    pointInfo.setIdentity();
    pointInfo *= 1000;
    pointSensor->setInformation(pointInfo);
    pointSensor->setFov(0.75 * M_PI);
    ss << "-pointXY";
  }

  if (hasPointBearingSensor) {
    auto* bearingSensor = new g2o::SensorPointXYBearing("bearingSensor");
    robot.addSensor(bearingSensor);
    bearingSensor->setInformation(bearingSensor->information() * 1000);
    ss << "-pointBearing";
  }

  if (hasSegmentSensor) {
    cerr << "creating Segment Sensor" << endl;
    auto* segmentSensor = new g2o::SensorSegment2D("segmentSensor");
    cerr << "segmentSensorCreated" << endl;
    segmentSensor->setMaxRange(3);
    segmentSensor->setMinRange(.1);
    robot.addSensor(segmentSensor);
    segmentSensor->setInformation(segmentSensor->information() * 1000);

    auto* segmentSensorLine =
        new g2o::SensorSegment2DLine("segmentSensorSensorLine");
    segmentSensorLine->setMaxRange(3);
    segmentSensorLine->setMinRange(.1);
    robot.addSensor(segmentSensorLine);
    g2o::Matrix2 m = segmentSensorLine->information();
    m = m * 1000;
    m(0, 0) *= 10;
    segmentSensorLine->setInformation(m);

    auto* segmentSensorPointLine =
        new g2o::SensorSegment2DPointLine("segmentSensorSensorPointLine");
    segmentSensorPointLine->setMaxRange(3);
    segmentSensorPointLine->setMinRange(.1);
    robot.addSensor(segmentSensorPointLine);
    g2o::Matrix3 m3 = segmentSensorPointLine->information();
    m3 = m3 * 1000;
    m3(2, 2) *= 10;
    segmentSensorPointLine->setInformation(m3);

    ss << "-segment2d";
  }

  robot.move(g2o::SE2());
  double pStraight = 0.7;
  g2o::SE2 moveStraight;
  moveStraight.setTranslation(g2o::Vector2(1., 0.));
  double pLeft = 0.15;
  g2o::SE2 moveLeft;
  moveLeft.setRotation(g2o::Rotation2D(M_PI / 2));
  // double pRight=0.15;
  g2o::SE2 moveRight;
  moveRight.setRotation(g2o::Rotation2D(-M_PI / 2));

  for (int i = 0; i < simSteps; i++) {
    cerr << "m";
    g2o::SE2 pose = robot.pose();
    double dtheta = -100;
    g2o::Vector2 dt;
    if (pose.translation().x() < -.5 * worldSize) {
      dtheta = 0;
    }

    if (pose.translation().x() > .5 * worldSize) {
      dtheta = -M_PI;
    }

    if (pose.translation().y() < -.5 * worldSize) {
      dtheta = M_PI / 2;
    }

    if (pose.translation().y() > .5 * worldSize) {
      dtheta = -M_PI / 2;
    }
    g2o::SE2 move;
    if (dtheta < -M_PI) {
      // select a random move of the robot
      double sampled = g2o::sampleUniform(0., 1., &generator);
      if (sampled < pStraight)
        move = moveStraight;
      else if (sampled < pStraight + pLeft)
        move = moveLeft;
      else
        move = moveRight;
    } else {
      double mTheta = dtheta - pose.rotation().angle();
      move.setRotation(g2o::Rotation2D(mTheta));
      if (move.rotation().angle() < std::numeric_limits<double>::epsilon()) {
        move.setTranslation(g2o::Vector2(1., 0.));
      }
    }
    robot.relativeMove(move);
    // do a sense
    cerr << "s";
    robot.sense();
  }
  // string fname=outputFilename + ss.str() + ".g2o";
  std::ofstream testStream(outputFilename.c_str());
  graph.save(testStream);

  return 0;
}
