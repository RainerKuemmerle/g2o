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

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>

#include "g2o/apps/g2o_simulator/sensor_odometry2d.h"
#include "g2o/apps/g2o_simulator/sensor_pointxy.h"
#include "g2o/apps/g2o_simulator/sensor_pointxy_bearing.h"
#include "g2o/apps/g2o_simulator/sensor_pose2d.h"
#include "g2o/apps/g2o_simulator/sensor_segment2d.h"
#include "g2o/apps/g2o_simulator/sensor_segment2d_line.h"
#include "g2o/apps/g2o_simulator/sensor_segment2d_pointline.h"
#include "g2o/apps/g2o_simulator/simulator.h"
#include "g2o/apps/g2o_simulator/simulator2d_base.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam2d/se2.h"

using std::cerr;

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
  g2o::World world;
  for (int i = 0; i < nlandmarks; i++) {
    auto landmark = std::make_unique<g2o::WorldObjectPointXY>();
    double x = g2o::sampleUniform(-.5, .5, &generator) * (worldSize + 5);
    double y = g2o::sampleUniform(-.5, .5, &generator) * (worldSize + 5);
    landmark->vertex()->setEstimate(g2o::Vector2(x, y));
    world.addWorldObject(std::move(landmark));
  }

  cerr << "nSegments = " << nSegments << '\n';

  for (int i = 0; i < nSegments; i++) {
    auto segment = std::make_unique<g2o::WorldObjectSegment2D>();
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
    world.addWorldObject(std::move(segment));
  }

  {
    auto robot = std::make_unique<g2o::Robot2D>("myRobot");

    if (hasOdom) {
      auto odometrySensor = std::make_unique<g2o::SensorOdometry2D>("odometry");
      const g2o::Vector3 diagonal(500, 500, 5000);
      odometrySensor->setInformation(diagonal.asDiagonal());
      robot->addSensor(std::move(odometrySensor), world);
    }

    if (hasPoseSensor) {
      auto poseSensor = std::make_unique<g2o::SensorPose2D>("poseSensor");
      g2o::Matrix3 poseInfo = poseSensor->information();
      poseInfo.setIdentity();
      poseInfo *= 500;
      poseInfo(2, 2) = 5000;
      poseSensor->setInformation(poseInfo);
      robot->addSensor(std::move(poseSensor), world);
    }

    if (hasPointSensor) {
      auto pointSensor = std::make_unique<g2o::SensorPointXY>("pointSensor");
      g2o::Matrix2 pointInfo = pointSensor->information();
      pointInfo.setIdentity();
      pointInfo *= 1000;
      pointSensor->setInformation(pointInfo);
      pointSensor->setFov(0.75 * M_PI);
      robot->addSensor(std::move(pointSensor), world);
    }

    if (hasPointBearingSensor) {
      auto bearingSensor =
          std::make_unique<g2o::SensorPointXYBearing>("bearingSensor");
      bearingSensor->setInformation(bearingSensor->information() * 1000);
      robot->addSensor(std::move(bearingSensor), world);
    }

    if (hasSegmentSensor) {
      cerr << "creating Segment Sensor\n";
      auto segmentSensor =
          std::make_unique<g2o::SensorSegment2D>("segmentSensor");
      cerr << "segmentSensorCreated\n";
      segmentSensor->setMaxRange(3);
      segmentSensor->setMinRange(.1);
      segmentSensor->setInformation(segmentSensor->information() * 1000);
      robot->addSensor(std::move(segmentSensor), world);

      auto segmentSensorLine =
          std::make_unique<g2o::SensorSegment2DLine>("segmentSensorSensorLine");
      segmentSensorLine->setMaxRange(3);
      segmentSensorLine->setMinRange(.1);
      g2o::Matrix2 m = segmentSensorLine->information();
      m = m * 1000;
      m(0, 0) *= 10;
      segmentSensorLine->setInformation(m);
      robot->addSensor(std::move(segmentSensorLine), world);

      auto segmentSensorPointLine =
          std::make_unique<g2o::SensorSegment2DPointLine>(
              "segmentSensorSensorPointLine");
      segmentSensorPointLine->setMaxRange(3);
      segmentSensorPointLine->setMinRange(.1);
      g2o::Matrix3 m3 = segmentSensorPointLine->information();
      m3 = m3 * 1000;
      m3(2, 2) *= 10;
      segmentSensorPointLine->setInformation(m3);
      robot->addSensor(std::move(segmentSensorPointLine), world);
    }

    world.addRobot(std::move(robot));
  }

  for (const auto& robot : world.robots()) {
    auto* rob2d = dynamic_cast<g2o::Robot2D*>(robot.get());
    if (!rob2d) continue;
    rob2d->move(world, g2o::SE2());
  }

  for (const auto& base_robot : world.robots()) {
    auto* robot = dynamic_cast<g2o::Robot2D*>(base_robot.get());
    if (!robot) continue;
    for (int i = 0; i < simSteps; i++) {
      cerr << "m";
      const g2o::SE2& pose = robot->pose();
      const std::optional<double> dtheta = [&]() -> std::optional<double> {
        if (pose.translation().x() < -.5 * worldSize) return 0.;
        if (pose.translation().x() > .5 * worldSize) return -M_PI;
        if (pose.translation().y() < -.5 * worldSize) return M_PI / 2;
        if (pose.translation().y() > .5 * worldSize) return -M_PI / 2;
        return std::nullopt;
      }();
      const g2o::SE2 move = [&]() {
        if (!dtheta.has_value()) {
          constexpr double kPStraight = 0.7;
          constexpr double kPLeft = 0.15;
          // select a random move of the robot
          double sampled = g2o::sampleUniform(0., 1., &generator);
          if (sampled < kPStraight) return g2o::SE2(1., 0., 0.);
          if (sampled < kPStraight + kPLeft) return g2o::SE2(0., 0., M_PI / 2);
          return g2o::SE2(0., 0., -M_PI / 2);
        }
        const double mTheta = dtheta.value() - pose.rotation().angle();
        g2o::SE2 result;
        result.setRotation(g2o::Rotation2D(mTheta));
        if (abs(move.rotation().angle()) <
            std::numeric_limits<double>::epsilon()) {
          result.setTranslation(g2o::Vector2(1., 0.));
        }
        return result;
      }();
      robot->relativeMove(world, move);
      // do a sense
      cerr << "s";
      robot->sense(world);
    }
  }

  std::ofstream testStream(outputFilename.c_str());
  world.graph().save(testStream);

  return 0;
}
