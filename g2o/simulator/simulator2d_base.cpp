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

#include "simulator2d_base.h"

#include <cmath>
#include <memory>
#include <optional>
#include <random>

#include "g2o/simulator/sensor_odometry2d.h"
#include "g2o/simulator/sensor_pointxy.h"
#include "g2o/simulator/sensor_pointxy_bearing.h"
#include "g2o/simulator/sensor_pose2d.h"
#include "g2o/simulator/sensor_se2_prior.h"
#include "g2o/simulator/sensor_segment2d.h"
#include "g2o/simulator/sensor_segment2d_line.h"
#include "g2o/simulator/sensor_segment2d_pointline.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam2d/se2.h"

namespace g2o {

Simulator2D::Simulator2D(Simulator2D::Config&& config) : config(config) {}

void Simulator2D::setup() {
  G2O_DEBUG("nlandmarks = {}", config.nlandmarks);
  for (int i = 0; i < config.nlandmarks; i++) {
    auto landmark = std::make_unique<WorldObjectPointXY>();
    double x = sampleUniform(-.5, .5, &generator_) * (config.worldSize + 5);
    double y = sampleUniform(-.5, .5, &generator_) * (config.worldSize + 5);
    landmark->vertex()->setEstimate(Vector2(x, y));
    world_.addWorldObject(std::move(landmark));
  }

  G2O_DEBUG("nSegments = {}", config.nSegments);
  for (int i = 0; i < config.nSegments; i++) {
    auto segment = std::make_unique<WorldObjectSegment2D>();
    int ix = sampleUniform(-config.segmentGridSize, config.segmentGridSize,
                           &generator_);
    int iy = sampleUniform(-config.segmentGridSize, config.segmentGridSize,
                           &generator_);
    int ith = sampleUniform(0, 3, &generator_);
    double th = (M_PI / 2) * ith;
    th = atan2(sin(th), cos(th));
    double xc = ix * (config.worldSize / config.segmentGridSize);
    double yc = iy * (config.worldSize / config.segmentGridSize);

    double l2 = sampleUniform(config.minSegmentLength, config.maxSegmentLength,
                              &generator_);

    double x1 = xc + cos(th) * l2;
    double y1 = yc + sin(th) * l2;
    double x2 = xc - cos(th) * l2;
    double y2 = yc - sin(th) * l2;

    segment->vertex()->setEstimateP1(Vector2(x1, y1));
    segment->vertex()->setEstimateP2(Vector2(x2, y2));
    world_.addWorldObject(std::move(segment));
  }

  auto robot = std::make_unique<Robot2D>("myRobot");

  if (config.hasOdom) {
    G2O_DEBUG("Adding odometry sensor");
    auto odometrySensor = std::make_unique<SensorOdometry2D>("odometry");
    const Vector3 diagonal(500, 500, 5000);
    odometrySensor->setInformation(diagonal.asDiagonal());
    robot->addSensor(std::move(odometrySensor), world_);
  }

  if (config.hasPoseSensor) {
    G2O_DEBUG("Adding pose sensor");
    auto poseSensor = std::make_unique<SensorPose2D>("poseSensor");
    Matrix3 poseInfo = poseSensor->information();
    poseInfo.setIdentity();
    poseInfo *= 500;
    poseInfo(2, 2) = 5000;
    poseSensor->setInformation(poseInfo);
    robot->addSensor(std::move(poseSensor), world_);
  }

  if (config.hasGPS) {
    G2O_DEBUG("Adding GPS sensor");
    auto gpsSensor = std::make_unique<SensorSE2Prior>("gpsSensor");
    robot->addSensor(std::move(gpsSensor), world_);
  }

  if (config.hasPointSensor) {
    G2O_DEBUG("Adding point sensor");
    auto pointSensor = std::make_unique<SensorPointXY>("pointSensor");
    Matrix2 pointInfo = pointSensor->information();
    pointInfo.setIdentity();
    pointInfo *= 1000;
    pointSensor->setInformation(pointInfo);
    pointSensor->setFov(0.75 * M_PI);
    robot->addSensor(std::move(pointSensor), world_);
  }

  if (config.hasPointBearingSensor) {
    G2O_DEBUG("Adding point bearing sensor");
    auto bearingSensor =
        std::make_unique<SensorPointXYBearing>("bearingSensor");
    bearingSensor->setInformation(bearingSensor->information() * 1000);
    robot->addSensor(std::move(bearingSensor), world_);
  }

  if (config.hasSegmentSensor) {
    G2O_DEBUG("Adding segment sensor");
    auto segmentSensor = std::make_unique<SensorSegment2D>("segmentSensor");
    segmentSensor->setMaxRange(3);
    segmentSensor->setMinRange(.1);
    segmentSensor->setInformation(segmentSensor->information() * 1000);
    robot->addSensor(std::move(segmentSensor), world_);

    auto segmentSensorLine =
        std::make_unique<SensorSegment2DLine>("segmentSensorSensorLine");
    segmentSensorLine->setMaxRange(3);
    segmentSensorLine->setMinRange(.1);
    Matrix2 m = segmentSensorLine->information();
    m = m * 1000;
    m(0, 0) *= 10;
    segmentSensorLine->setInformation(m);
    robot->addSensor(std::move(segmentSensorLine), world_);

    auto segmentSensorPointLine = std::make_unique<SensorSegment2DPointLine>(
        "segmentSensorSensorPointLine");
    segmentSensorPointLine->setMaxRange(3);
    segmentSensorPointLine->setMinRange(.1);
    Matrix3 m3 = segmentSensorPointLine->information();
    m3 = m3 * 1000;
    m3(2, 2) *= 10;
    segmentSensorPointLine->setInformation(m3);
    robot->addSensor(std::move(segmentSensorPointLine), world_);
  }

  world_.addRobot(std::move(robot));
}

void Simulator2D::simulate() {
  const Config& sim_conf = config;
  std::mt19937& generator = generator_;

  G2O_DEBUG("Simulate {} steps in world size {}", sim_conf.simSteps,
            sim_conf.worldSize);

  for (const auto& robot : world_.robots()) {
    auto* rob2d = dynamic_cast<Robot2D*>(robot.get());
    if (!rob2d) continue;
    rob2d->move(world_, SE2());
  }

  for (const auto& base_robot : world_.robots()) {
    auto* robot = dynamic_cast<Robot2D*>(base_robot.get());
    if (!robot) continue;
    for (int i = 0; i < config.simSteps; i++) {
      G2O_TRACE("move");
      const SE2& pose = robot->pose();
      const std::optional<double> dtheta =
          [&pose, &sim_conf]() -> std::optional<double> {
        if (pose.translation().x() < -.5 * sim_conf.worldSize) return 0.;
        if (pose.translation().x() > .5 * sim_conf.worldSize) return -M_PI;
        if (pose.translation().y() < -.5 * sim_conf.worldSize) return M_PI / 2;
        if (pose.translation().y() > .5 * sim_conf.worldSize) return -M_PI / 2;
        return std::nullopt;
      }();
      const SE2 move = [&pose, &dtheta, &generator]() {
        if (!dtheta.has_value()) {
          constexpr double kPStraight = 0.7;
          constexpr double kPLeft = 0.15;
          // select a random move of the robot
          double sampled = sampleUniform(0., 1., &generator);
          if (sampled < kPStraight) return SE2(1., 0., 0.);
          if (sampled < kPStraight + kPLeft) return SE2(0., 0., M_PI / 2);
          return SE2(0., 0., -M_PI / 2);
        }
        const double mTheta = dtheta.value() - pose.rotation().angle();
        SE2 result;
        result.setRotation(Rotation2D(mTheta));
        if (abs(result.rotation().angle()) <
            std::numeric_limits<double>::epsilon()) {
          result.setTranslation(Vector2(1., 0.));
        }
        return result;
      }();
      robot->relativeMove(world_, move);
      // do a sense
      G2O_TRACE("sense");
      robot->sense(world_);
    }
  }
  finalize();
}

}  // namespace g2o
